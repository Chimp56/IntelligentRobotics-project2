#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import re
import math
import itertools
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# We use feet everywhere to match the assignment and your other nodes
FEET_PER_METER = 3.28084

TASK_LINE_RE = re.compile(
    r"\(\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*,\s*\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*\)"
)

class TaskPlanner(object):
    def __init__(self):
        rospy.init_node('task_planner', anonymous=True)

        # --- Params ---
        # Path to a text file whose lines look like: ((2, 3), (9, 8))
        self.tasks_file = rospy.get_param("~tasks_file", "")
        # Alternatively, accept raw text via param ~tasks_text (one task per line)
        self.tasks_text = rospy.get_param("~tasks_text", "")

        # --- ROS I/O ---
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_cb)
        self.status_sub = rospy.Subscriber("/execution_monitor/status", String, self._status_cb)
        self.waypoints_pub = rospy.Publisher("/task_planner/waypoints", String, queue_size=1, latch=True)

        # --- State ---
        self.startup_origin_m = None   # (x,y) meters when we first see odom
        self.startup_origin_ft = (0.0, 0.0)
        self.current_pose_ft = None

        # Each task: {"id": k, "start": (sx,sy), "dest": (dx,dy)}
        self.tasks = []
        # Flattened best sequence of points (each element: dict with keys: "id","kind","pt")
        self.best_sequence = []
        self.remaining_sequence = []   # remaining points not yet attempted
        self.current_index = 0         # index into full sequence of the point were attempting
        self.republished_after_failure = False

        rospy.loginfo("TaskPlanner: initialized.")

    # ------------ Public run ------------
    def run(self):
        # 1) Load tasks
        if not self._load_tasks():
            rospy.logerr("TaskPlanner: No tasks found (tasks_file or tasks_text). Exiting.")
            return

        # 2) Wait for odom once so our cost includes distance from the robot start
        rospy.loginfo("TaskPlanner: waiting for first odom to set origin...")
        while not rospy.is_shutdown() and self.current_pose_ft is None:
            rospy.sleep(0.05)

        # 3) Compute best sequence obeying precedence constraints
        self.best_sequence = self._compute_best_sequence(self.tasks)
        self.remaining_sequence = list(self.best_sequence)
        rospy.loginfo("TaskPlanner: planned %d points.", len(self.best_sequence))

        # 4) Publish the full sequence to NavigationController
        self._publish_remaining_sequence()

        # Spin while we listen for monitor status (SUCCESS/FAILURE)
        rospy.loginfo("TaskPlanner: running; listening for execution monitor updates.")
        rospy.spin()

    # ------------ ROS callbacks ------------
    def _odom_cb(self, msg):
        px_m = msg.pose.pose.position.x
        py_m = msg.pose.pose.position.y
        if self.startup_origin_m is None:
            self.startup_origin_m = (px_m, py_m)
            rospy.loginfo("TaskPlanner: origin set at odom (%.3f, %.3f) m", px_m, py_m)
        # convert to feet with origin at startup
        dx_m = px_m - self.startup_origin_m[0]
        dy_m = py_m - self.startup_origin_m[1]
        self.current_pose_ft = (dx_m * FEET_PER_METER, dy_m * FEET_PER_METER)

    def _status_cb(self, msg):
        """
        We watch for SUCCESS / FAILURE to keep our index aligned and to handle the
        special rule:
          - If FAILURE on a **start** point, remove that tasks **destination** from the plan.
        NavigationController already advances on SUCCESS or FAILURE, so we only need
        to (a) track our current index and (b) possibly re-publish a shortened plan.
        """
        text = msg.data.strip()
        if not self.remaining_sequence:
            return

        if text.startswith("SUCCESS"):
            # We reached the current target; advance pointer.
            if self.remaining_sequence:
                self._pop_front()  # drop the one just reached
            self.republished_after_failure = False

        elif text.startswith("FAILURE"):
            # If the failed point was a START, drop its DEST from whatever remains.
            if not self.remaining_sequence:
                return
            failed = self.remaining_sequence[0]  # this is the point that NavController just attempted
            # Advance past the failed point either way
            failed_elem = self._pop_front()

            if failed_elem["kind"] == "START":
                # Remove the matching destination from the rest of the sequence, if it still exists
                task_id = failed_elem["id"]
                before = len(self.remaining_sequence)
                self.remaining_sequence = [e for e in self.remaining_sequence
                                           if not (e["id"] == task_id and e["kind"] == "DEST")]
                after = len(self.remaining_sequence)
                if before != after:
                    rospy.logwarn("TaskPlanner: removed DEST for task %s due to start failure.", task_id)
                # Re-publish the updated remainder so NavController resets to the new list
                self._publish_remaining_sequence()
                self.republished_after_failure = True

        # We ignore PROGRESS / STALLED here.

    # ------------ Helpers ------------
    def _pop_front(self):
        """Pop the element at the front of remaining_sequence and return it."""
        if not self.remaining_sequence:
            return None
        elem = self.remaining_sequence.pop(0)
        return elem

    def _publish_remaining_sequence(self):
        """
        Publish remaining_sequence as 'x,y; x,y; ...' (feet).
        NavigationController will call set_waypoints() and start driving.
        """
        s = "; ".join(["{:.3f},{:.3f}".format(e["pt"][0], e["pt"][1]) for e in self.remaining_sequence])
        self.waypoints_pub.publish(String(data=s))
        rospy.loginfo("TaskPlanner: published %d remaining waypoints.", len(self.remaining_sequence))

    def _load_tasks(self):
        lines = []
        if self.tasks_text.strip():
            lines = [ln for ln in self.tasks_text.strip().splitlines() if ln.strip()]
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
                rospy.logwarn("TaskPlanner: skipping unparsable line %d: %s", i+1, ln.rstrip())
                continue
            sx, sy, dx, dy = map(float, m.groups())
            tasks.append({"id": i, "start": (sx, sy), "dest": (dx, dy)})

        self.tasks = tasks
        if not tasks:
            return False
        rospy.loginfo("TaskPlanner: loaded %d tasks.", len(tasks))
        return True

    # ---------- Planning: precedence-constrained shortest sequence ----------
    def _compute_best_sequence(self, tasks):
        """
        Return a list of dicts: [{"id":k,"kind":"START"/"DEST","pt":(x,y)}, ...]
        that minimizes straight-line travel distance, subject to START < DEST
        for each task. Start point for cost is the robot’s (0,0) feet at startup.
        (Python 2 compatible: no 'nonlocal'.)
        """
        # Build list of (kind, task_id, point)
        elements = []
        for t in tasks:
            elements.append(("START", t["id"], t["start"]))
            elements.append(("DEST",  t["id"], t["dest"]))

        # Holders replace 'nonlocal' vars
        best_seq_holder = [None]
        best_cost_holder = [float('inf')]

        def valid_to_add(partial, cand):
            kind, tid, _ = cand
            if kind == "DEST":
                # Only place DEST after its START is already in partial
                for k, t, _pt in partial:
                    if k == "START" and t == tid:
                        return True
                return False
            return True

        def cost_of(sequence):
            if not sequence:
                return 0.0
            cost = 0.0
            prev = (0.0, 0.0)  # robot origin in feet
            for _k, _t, pt in sequence:
                cost += math.hypot(pt[0] - prev[0], pt[1] - prev[1])
                prev = pt
            return cost

        def backtrack(partial, remaining):
            # If nothing remains, evaluate this complete sequence
            if not remaining:
                c = cost_of(partial)
                if c < best_cost_holder[0]:
                    best_cost_holder[0] = c
                    best_seq_holder[0] = list(partial)
                return

            # Small heuristic: try nearer points first to speed search
            rem_sorted = sorted(remaining, key=lambda e: math.hypot(e[2][0], e[2][1]))
            for i, cand in enumerate(rem_sorted):
                if not valid_to_add(partial, cand):
                    continue
                next_remaining = rem_sorted[:i] + rem_sorted[i+1:]
                partial.append(cand)

                # Cheap pruning: if partial cost already exceeds best, skip
                if best_seq_holder[0] is None or cost_of(partial) <= best_cost_holder[0]:
                    backtrack(partial, next_remaining)

                partial.pop()

        backtrack([], elements)

        seq = [{"id": tid, "kind": kind, "pt": (pt[0], pt[1])}
            for (kind, tid, pt) in best_seq_holder[0]]
        return seq

        

if __name__ == "__main__":
    try:
        TaskPlanner().run()
    except rospy.ROSInterruptException:
        pass
