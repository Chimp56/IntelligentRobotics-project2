#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import re
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# feet/meter conversion
FEET_PER_METER = 3.28084

# Parse lines like: ((2, 3), (9, 8))
TASK_LINE_RE = re.compile(
    r"\(\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*,\s*\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*\)"
)

class TaskPlanner(object):
    def __init__(self):
        rospy.init_node('task_planner', anonymous=False)

        # -------- Params --------
        # Incoming task list (pairs of (start,dest) points in feet)
        self.tasks_file = rospy.get_param("~tasks_file", "")
        self.tasks_text = rospy.get_param("~tasks_text", "")

        # Robot's starting pose in the assignment's global frame (feet + deg).
        # This fulfills: "provide your robot with its starting pose (location and orientation)
        # when you start the simulation."
        self.start_x_feet    = rospy.get_param("~start_x_feet", 0.0)
        self.start_y_feet    = rospy.get_param("~start_y_feet", 0.0)
        self.start_theta_deg = rospy.get_param("~start_theta_deg", 0.0)
        self.start_pose_feet = (self.start_x_feet,
                                self.start_y_feet,
                                self.start_theta_deg)

        # -------- ROS I/O --------
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb)
        self.status_sub = rospy.Subscriber("/execution_monitor/status",
                                           String, self._status_cb)

        # Latched publisher so nav can join late
        self.waypoints_pub = rospy.Publisher("/task_planner/waypoints",
                                             String, queue_size=1, latch=True)

        # -------- Internal State --------
        self.startup_origin_m = None   # (mx,my) at first odom, for runtime tracking
        self.current_pose_ft = None    # robot pose in FEET (absolute world coordinates)

        # Tasks: [{"id":i, "start":(sx,sy), "dest":(dx,dy)}, ...]
        self.tasks = []

        # Full best plan: [{"id":k, "kind":"START"/"DEST", "pt":(x,y)}, ...]
        self.best_sequence = []

        # What remains to attempt (mutates as we succeed/fail)
        self.remaining_sequence = []

        self.current_index = 0
        self.republished_after_failure = False

        rospy.loginfo("TaskPlanner: initialized.")
        rospy.loginfo("TaskPlanner: given start pose feet = (%.2f, %.2f) heading %.1f deg",
                      self.start_x_feet, self.start_y_feet, self.start_theta_deg)

    def run(self):
        # 1. Load tasks
        if not self._load_tasks():
            rospy.logerr("TaskPlanner: No tasks found (tasks_file or tasks_text). Exiting.")
            return

        # 2. Wait for odom so we're synced with the sim
        rospy.loginfo("TaskPlanner: waiting for first odom to set origin...")
        while not rospy.is_shutdown() and self.current_pose_ft is None:
            rospy.sleep(0.05)

        # 3. Compute best visiting order
        self.best_sequence = self._compute_best_sequence(self.tasks)
        self.remaining_sequence = list(self.best_sequence)
        rospy.loginfo("TaskPlanner: planned %d points.", len(self.best_sequence))

        # 4. Publish initial waypoint list for navigation_controller
        self._publish_remaining_sequence()

        # 5. Spin and react to execution monitor status
        rospy.loginfo("TaskPlanner: running; listening for execution monitor updates.")
        rospy.spin()

    # -------- Callbacks --------
    def _odom_cb(self, msg):
        px_m = msg.pose.pose.position.x
        py_m = msg.pose.pose.position.y

        if self.startup_origin_m is None:
            # This defines a *local* odom reference frame origin for displacement tracking.
            self.startup_origin_m = (px_m, py_m)
            rospy.loginfo("TaskPlanner: odom reference frame origin set at (%.3f, %.3f) m",
                          px_m, py_m)
            rospy.loginfo("TaskPlanner: robot actual world position (%.2f, %.2f) feet",
                          self.start_x_feet, self.start_y_feet)

        # Calculate displacement from reference origin, convert to feet, then add starting offset
        dx_m = px_m - self.startup_origin_m[0]
        dy_m = py_m - self.startup_origin_m[1]
        self.current_pose_ft = (dx_m * FEET_PER_METER + self.start_x_feet, 
                                dy_m * FEET_PER_METER + self.start_y_feet)

    def _status_cb(self, msg):
        """
        ExecutionMonitor reports:
          SUCCESS: reached target
          FAILURE: couldn't reach target
          PROGRESS/STALLED: info only

        Rules from assignment:
        - If FAILURE happens on a START point for a task, we must remove that task's DEST
          from the rest of the plan.
        """
        text = msg.data.strip()
        if not self.remaining_sequence:
            return

        if text.startswith("SUCCESS"):
            # Drop the point we just hit
            if self.remaining_sequence:
                self._pop_front()
            self.republished_after_failure = False

        elif text.startswith("FAILURE"):
            if not self.remaining_sequence:
                return

            failed_elem = self._pop_front()
            if failed_elem and failed_elem["kind"] == "START":
                task_id = failed_elem["id"]

                # Remove that task's DEST from whatever remains
                before = len(self.remaining_sequence)
                self.remaining_sequence = [
                    e for e in self.remaining_sequence
                    if not (e["id"] == task_id and e["kind"] == "DEST")
                ]
                after = len(self.remaining_sequence)
                if before != after:
                    rospy.logwarn("TaskPlanner: removed DEST for task %s due to start failure.",
                                  task_id)

                # Re-publish shortened plan so nav realigns
                self._publish_remaining_sequence()
                self.republished_after_failure = True

        # Ignore PROGRESS / STALLED for planning purposes

    # -------- Helpers --------
    def _pop_front(self):
        if not self.remaining_sequence:
            return None
        elem = self.remaining_sequence.pop(0)
        return elem

    def _publish_remaining_sequence(self):
        """
        Publish remaining target list as
          'x,y; x,y; x,y'
        all in FEET.
        """
        s = "; ".join(
            ["{:.3f},{:.3f}".format(e["pt"][0], e["pt"][1])
             for e in self.remaining_sequence]
        )
        self.waypoints_pub.publish(String(data=s))
        rospy.loginfo("TaskPlanner: published %d remaining waypoints.",
                      len(self.remaining_sequence))

    def _load_tasks(self):
        """
        Parse the provided task list into self.tasks:
          [{"id": i, "start": (sx,sy), "dest": (dx,dy)}, ...]
        Units: FEET.
        """
        lines = []
        if self.tasks_text.strip():
            lines = [ln for ln in self.tasks_text.strip().splitlines()
                     if ln.strip()]
        elif self.tasks_file:
            try:
                with open(self.tasks_file, 'r') as f:
                    lines = [ln for ln in f.readlines() if ln.strip()]
            except Exception as e:
                rospy.logerr("TaskPlanner: failed to read tasks_file: %s", e)
                return False
        else:
            return False

        tasks = []
        for i, ln in enumerate(lines):
            m = TASK_LINE_RE.search(ln)
            if not m:
                rospy.logwarn("TaskPlanner: skipping unparsable line %d: %s",
                              i + 1, ln.rstrip())
                continue
            sx, sy, dx, dy = map(float, m.groups())
            tasks.append({
                "id": i,
                "start": (sx, sy),
                "dest":  (dx, dy)
            })

        self.tasks = tasks
        if not tasks:
            return False

        rospy.loginfo("TaskPlanner: loaded %d tasks.", len(tasks))
        return True

    # -------- Planner / Ordering --------
    def _compute_best_sequence(self, tasks):
        """
        We build a visit order of all START and DEST points that:
        - Obeys: each task's START must come before that task's DEST
        - Minimizes total straight-line travel distance
        - Includes distance from the robot's PROVIDED starting pose
          (start_x_feet, start_y_feet) to the first chosen point

        Output format:
        [
          {"id": task_id, "kind": "START" or "DEST", "pt": (x,y)},
          ...
        ]
        """
        # Build (kind, task_id, point)
        candidates = []
        for t in tasks:
            candidates.append(("START", t["id"], t["start"]))
            candidates.append(("DEST",  t["id"], t["dest"]))

        best_seq_holder = [None]
        best_cost_holder = [float('inf')]

        def valid_to_add(partial, cand):
            kind, tid, _pt = cand
            if kind == "DEST":
                # can't add DEST unless START for this tid is already in partial
                for k, t_id, _p in partial:
                    if k == "START" and t_id == tid:
                        return True
                return False
            return True

        def path_cost(sequence):
            """
            Total travel if we begin from the robot's provided start pose.
            """
            if not sequence:
                return 0.0

            prev_x = self.start_pose_feet[0]
            prev_y = self.start_pose_feet[1]
            total = 0.0
            for (_kind, _tid, pt) in sequence:
                dx = pt[0] - prev_x
                dy = pt[1] - prev_y
                total += math.hypot(dx, dy)
                prev_x, prev_y = pt[0], pt[1]
            return total

        def backtrack(partial, remaining):
            # If no more points to assign, evaluate this ordering
            if not remaining:
                c = path_cost(partial)
                if c < best_cost_holder[0]:
                    best_cost_holder[0] = c
                    best_seq_holder[0] = list(partial)
                return

            # small heuristic: consider closer points (to the robot's start pose)
            rem_sorted = sorted(
                remaining,
                key=lambda e: math.hypot(
                    e[2][0] - self.start_pose_feet[0],
                    e[2][1] - self.start_pose_feet[1]
                )
            )

            for i, cand in enumerate(rem_sorted):
                if not valid_to_add(partial, cand):
                    continue

                next_remaining = rem_sorted[:i] + rem_sorted[i+1:]
                partial.append(cand)

                # prune if already worse than best complete we've seen
                if (best_seq_holder[0] is None or
                        path_cost(partial) <= best_cost_holder[0]):
                    backtrack(partial, next_remaining)

                partial.pop()

        # Kick off the backtracking search
        backtrack([], candidates)

        # Convert tuples -> dict format we use downstream
        seq = [
            {"id": tid, "kind": kind, "pt": (pt[0], pt[1])}
            for (kind, tid, pt) in best_seq_holder[0]
        ]
        return seq


if __name__ == "__main__":
    try:
        planner = TaskPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass
