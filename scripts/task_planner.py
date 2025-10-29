#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import re
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# We use feet everywhere to match the assignment and other nodes
FEET_PER_METER = 3.28084

# Regex to parse lines like: ((2, 3), (9, 8))
TASK_LINE_RE = re.compile(
    r"\(\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*,\s*\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*\)"
)

class TaskPlanner(object):
    def __init__(self):
        rospy.init_node('task_planner', anonymous=False)

        # --- Params ---
        # Option A: tasks from file
        self.tasks_file = rospy.get_param("~tasks_file", "")
        # Option B: tasks from launch param inline
        self.tasks_text_param = rospy.get_param("~tasks_text", "")

        # robot spawn pose in global feet coords (given by launch)
        self.spawn_x_ft = rospy.get_param("~start_x_feet", 0.0)
        self.spawn_y_ft = rospy.get_param("~start_y_feet", 0.0)
        self.spawn_theta_deg = rospy.get_param("~start_theta_deg", 0.0)

        # --- ROS I/O ---
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb)
        self.status_sub = rospy.Subscriber("/execution_monitor/status",
                                           String, self._status_cb)

        # publish current plan (semicolon-separated x,y list in FEET)
        self.waypoints_pub = rospy.Publisher("/task_planner/waypoints",
                                             String,
                                             queue_size=1,
                                             latch=True)

        # listen for NEW task sets at runtime
        self.new_tasks_sub = rospy.Subscriber("/task_planner/new_task_text",
                                              String,
                                              self._new_tasks_callback)

        # --- State ---
        # robot odom origin in meters when we first heard odom
        self.odom_zero_m = None  # (x_m, y_m)

        # robot current position (feet, world frame consistent with spawn)
        self.current_pose_ft = None

        # tasks: list of dicts {"id": k, "start": (sx,sy), "dest": (dx,dy)}
        self.tasks = []

        # full best path = list of dicts like
        # {"id": tid, "kind":"START"/"DEST", "pt": (x_ft, y_ft)}
        self.best_sequence = []

        # what’s left to visit after reacting to monitor updates
        self.remaining_sequence = []

        # book-keeping for reacting to FAILURE rules
        self.republished_after_failure = False

        rospy.loginfo("TaskPlanner: initialized.")
        rospy.loginfo("Spawn pose feet (%.2f, %.2f) yaw %.1f deg",
                      self.spawn_x_ft, self.spawn_y_ft, self.spawn_theta_deg)

    # ------------------------------------------------------------------
    # main loop
    # ------------------------------------------------------------------
    def run(self):
        # 1) Load initial tasks from launch or file
        if not self._load_tasks_from_sources():
            rospy.logerr("TaskPlanner: No initial tasks. Exiting.")
            return

        # 2) Wait for first odom so we can track robot pose if needed
        rospy.loginfo("TaskPlanner: waiting for first odom...")
        while not rospy.is_shutdown() and self.current_pose_ft is None:
            rospy.sleep(0.05)

        # 3) Plan with those tasks
        self._plan_and_publish()

        # 4) spin: we now react to
        #    - execution monitor feedback (SUCCESS / FAILURE)
        #    - new task sets from /task_planner/new_task_text
        rospy.loginfo("TaskPlanner: running; listening for updates.")
        rospy.spin()

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _odom_cb(self, msg):
        """
        Track robot position in *world feet coords* based on:
          - Gazebo spawn feet (from launch)
          - odom drift from start in meters
        """
        px_m = msg.pose.pose.position.x
        py_m = msg.pose.pose.position.y

        if self.odom_zero_m is None:
            self.odom_zero_m = (px_m, py_m)
            rospy.loginfo("TaskPlanner: odom_zero_m set at (%.3f, %.3f) m",
                          px_m, py_m)

        # offset from when we started
        dx_m = px_m - self.odom_zero_m[0]
        dy_m = py_m - self.odom_zero_m[1]

        # convert that delta to feet, then add spawn feet offset
        dx_ft = dx_m * FEET_PER_METER
        dy_ft = dy_m * FEET_PER_METER

        self.current_pose_ft = (self.spawn_x_ft + dx_ft,
                                self.spawn_y_ft + dy_ft)

    def _status_cb(self, msg):
        """
        React to execution monitor.
        - SUCCESS => advance to next waypoint
        - FAILURE => if we failed a START for task X, drop DEST for X
                     and republish updated plan
        """
        text = msg.data.strip()
        if not self.remaining_sequence:
            return

        if text.startswith("SUCCESS"):
            # Robot reached the current waypoint.
            self._pop_front()
            self.republished_after_failure = False

        elif text.startswith("FAILURE"):
            if not self.remaining_sequence:
                return

            failed_elem = self._pop_front()  # remove the one we just tried

            if failed_elem and failed_elem["kind"] == "START":
                # Drop that task's DEST from whatever is left
                task_id = failed_elem["id"]
                before = len(self.remaining_sequence)
                self.remaining_sequence = [
                    e for e in self.remaining_sequence
                    if not (e["id"] == task_id and e["kind"] == "DEST")
                ]
                after = len(self.remaining_sequence)
                if before != after:
                    rospy.logwarn("TaskPlanner: removed DEST for task %s due to START failure.",
                                  task_id)

                # Re-publish updated remainder so NavigationController
                # can continue with what’s left
                self._publish_remaining_sequence()
                self.republished_after_failure = True

        # We ignore PROGRESS / STALLED here.

    def _new_tasks_callback(self, msg):
        """
        Accepts either line-separated or semicolon-separated tasks, e.g.:
        ((8,2),(6,7))
        ((3,5),(2,1))
        or: ((8,2),(6,7)); ((3,5),(2,1))
        """
        txt = msg.data.strip()
        if not txt:
            rospy.logwarn("TaskPlanner: received empty new_task_text.")
            return

        # unify separators: split on newlines and semicolons
        raw_chunks = []
        for ln in txt.replace('&#10;', '\n').splitlines():
            raw_chunks.extend([c.strip() for c in ln.split(';') if c.strip()])

        new_tasks = []
        for i, chunk in enumerate(raw_chunks):
            m = TASK_LINE_RE.search(chunk)
            if not m:
                rospy.logwarn("TaskPlanner: skipping bad task %d: %s", i+1, chunk)
                continue
            sx, sy, dx, dy = map(float, m.groups())
            new_tasks.append({"id": i, "start": (sx, sy), "dest": (dx, dy)})

        if not new_tasks:
            rospy.logwarn("TaskPlanner: parsed 0 valid new tasks.")
            return

        rospy.loginfo("TaskPlanner: parsed %d new tasks; re-planning.", len(new_tasks))
        self.tasks = new_tasks
        self._plan_and_publish()

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _plan_and_publish(self):
        """
        Compute best_sequence from self.tasks,
        reset remaining_sequence to that,
        and publish via /task_planner/waypoints.
        """
        self.best_sequence = self._compute_best_sequence(self.tasks)
        self.remaining_sequence = list(self.best_sequence)

        rospy.loginfo("TaskPlanner: planned %d waypoints.", len(self.best_sequence))
        self._publish_remaining_sequence()

    def _pop_front(self):
        """
        Pop the element at front of remaining_sequence and return it.
        """
        if not self.remaining_sequence:
            return None
        elem = self.remaining_sequence.pop(0)
        return elem

    def _publish_remaining_sequence(self):
        """
        Publish remaining_sequence as:
           "x,y; x,y; x,y"
        in FEET.
        NavigationController will parse and start moving.
        """
        s = "; ".join(
            ["{:.3f},{:.3f}".format(e["pt"][0], e["pt"][1])
             for e in self.remaining_sequence]
        )
        self.waypoints_pub.publish(String(data=s))
        rospy.loginfo("TaskPlanner: published %d remaining waypoints.", len(self.remaining_sequence))

    def _load_tasks_from_sources(self):
        """
        Called once at startup.
        Prefer ~tasks_text param if provided, else ~tasks_file.
        Returns True if we got >=1 valid task.
        """
        lines = []
        if self.tasks_text_param.strip():
            # tasks_text param may contain literal "\n" or XML &#10; newlines
            txt = self.tasks_text_param.replace('&#10;', '\n')
            for ln in txt.splitlines():
                if ln.strip():
                    lines.append(ln.strip())
        elif self.tasks_file:
            try:
                with open(self.tasks_file, 'r') as f:
                    for ln in f.readlines():
                        if ln.strip():
                            lines.append(ln.strip())
            except Exception as e:
                rospy.logerr("TaskPlanner: failed to read tasks_file: %s", e)
                return False
        else:
            return False

        self.tasks = self._parse_task_lines(lines)
        if not self.tasks:
            return False

        rospy.loginfo("TaskPlanner: loaded %d tasks at startup.", len(self.tasks))
        return True

    def _parse_task_lines(self, lines):
        """
        Take lines like:
            ((2, 3), (9, 8))
            ((12, 9), (4, 14))
        Return list of {"id":k, "start":(sx,sy), "dest":(dx,dy)}.
        """
        tasks = []
        for i, ln in enumerate(lines):
            m = TASK_LINE_RE.search(ln)
            if not m:
                rospy.logwarn("TaskPlanner: skipping bad line %d: %s", i+1, ln)
                continue
            sx, sy, dx, dy = map(float, m.groups())
            tasks.append({"id": i, "start": (sx, sy), "dest": (dx, dy)})
        return tasks

    # ------------------------------------------------------------------
    # planning search
    # ------------------------------------------------------------------
    def _compute_best_sequence(self, tasks):
        """
        Return a list of dicts:
          [{"id":k,"kind":"START"/"DEST","pt":(x,y)}, ...]
        that minimizes straight-line travel distance,
        subject to precedence (START must come before DEST for each task).

        Cost starts at the robot spawn position (spawn_x_ft, spawn_y_ft),
        not always (0,0), which satisfies the project requirement
        "provide your robot with its starting pose".
        """
        # Elements we need to order:
        #   ("START", task_id, (sx,sy)), ("DEST", task_id, (dx,dy))
        elements = []
        for t in tasks:
            elements.append(("START", t["id"], t["start"]))
            elements.append(("DEST",  t["id"], t["dest"]))

        best_seq_holder = [None]
        best_cost_holder = [float('inf')]

        def valid_to_add(partial, cand):
            kind, tid, _ = cand
            if kind == "DEST":
                # A DEST for task tid can only appear if we've already placed
                # that same task's START in partial
                for (k_partial, tid_partial, _pt_partial) in partial:
                    if k_partial == "START" and tid_partial == tid:
                        return True
                return False
            return True

        def cost_of(sequence):
            if not sequence:
                return 0.0
            total = 0.0
            # start cost from the robot's spawn position (feet)
            prev = (self.spawn_x_ft, self.spawn_y_ft)
            for (_k, _tid, pt) in sequence:
                total += math.hypot(pt[0] - prev[0], pt[1] - prev[1])
                prev = pt
            return total

        def backtrack(partial, remaining):
            # all placed?
            if not remaining:
                c = cost_of(partial)
                if c < best_cost_holder[0]:
                    best_cost_holder[0] = c
                    best_seq_holder[0] = list(partial)
                return

            # heuristic: try nearest next points first (distance from spawn, cheap-ish)
            rem_sorted = sorted(
                remaining,
                key=lambda e: math.hypot(e[2][0] - self.spawn_x_ft,
                                         e[2][1] - self.spawn_y_ft)
            )

            for i, cand in enumerate(rem_sorted):
                if not valid_to_add(partial, cand):
                    continue

                next_remaining = rem_sorted[:i] + rem_sorted[i+1:]
                partial.append(cand)

                # prune if already worse than best
                if (best_seq_holder[0] is None) or (cost_of(partial) <= best_cost_holder[0]):
                    backtrack(partial, next_remaining)

                partial.pop()

        backtrack([], elements)

        # convert [("START", id, (x,y)), ("DEST", id, (x,y)), ...]
        # into [{"id":id, "kind":"START", "pt":(x,y)}, ...]
        out = []
        if best_seq_holder[0] is not None:
            for (kind, tid, pt) in best_seq_holder[0]:
                out.append({"id": tid,
                            "kind": kind,
                            "pt": (pt[0], pt[1])})
        return out


if __name__ == "__main__":
    try:
        TaskPlanner().run()
    except rospy.ROSInterruptException:
        pass
