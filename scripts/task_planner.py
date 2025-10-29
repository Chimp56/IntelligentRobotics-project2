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
    r"\(\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*,\s*"
    r"\(\s*([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*\)\s*\)"
)

class TaskPlanner(object):
    def __init__(self):
        rospy.init_node('task_planner', anonymous=False)

        # -------- Params (startup defaults) --------
        # Option 1: read tasks from a param/launch file
        self.tasks_file = rospy.get_param("~tasks_file", "")
        self.tasks_text_param = rospy.get_param("~tasks_text", "")

        # Robot's starting pose in global assignment coordinates (feet + deg)
        # This is the "starting pose provided to the robot" per the spec.
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

        # NEW: listen for new task lists at runtime
        # User can do:
        #   rostopic pub /task_planner/new_tasks_text std_msgs/String \
        #   "data: '((5, 1), (9, 8))\n((12, 9), (4, 14))'"
        self.new_tasks_sub = rospy.Subscriber("/task_planner/new_tasks_text",
                                              String,
                                              self._new_tasks_cb)

        # Latched publisher: nav can start later and still see the plan
        self.waypoints_pub = rospy.Publisher("/task_planner/waypoints",
                                             String,
                                             queue_size=1,
                                             latch=True)

        # -------- Internal State --------
        self.startup_origin_m = None   # (mx,my) at first odom, for runtime tracking
        self.current_pose_ft = None    # robot pose in FEET relative to startup_origin_m

        # Tasks: [{"id":i, "start":(sx,sy), "dest":(dx,dy)}, ...]
        self.tasks = []

        # Full best plan sequence:
        # [{"id":k, "kind":"START"/"DEST", "pt":(x,y)}, ...]
        self.best_sequence = []

        # Remaining points to attempt (mutates as we go)
        self.remaining_sequence = []

        # bookkeeping for syncing with execution monitor
        self.republished_after_failure = False

        rospy.loginfo("TaskPlanner: initialized.")
        rospy.loginfo("TaskPlanner: given start pose feet = (%.2f, %.2f) heading %.1f deg",
                      self.start_x_feet, self.start_y_feet, self.start_theta_deg)

    # ------------------------------------------------------------------
    # Public main loop
    # ------------------------------------------------------------------
    def run(self):
        # initial plan from startup params
        if not self._plan_from_text_or_file(self.tasks_text_param, self.tasks_file):
            rospy.logerr("TaskPlanner: No initial tasks found (tasks_file or tasks_text). Waiting for new tasks on /task_planner/new_tasks_text.")
        else:
            rospy.loginfo("TaskPlanner: initial plan published.")

        rospy.loginfo("TaskPlanner: running; listening for execution monitor updates and new tasks.")
        rospy.spin()

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------
    def _odom_cb(self, msg):
        px_m = msg.pose.pose.position.x
        py_m = msg.pose.pose.position.y

        if self.startup_origin_m is None:
            # runtime reference frame for feet conversion
            self.startup_origin_m = (px_m, py_m)
            rospy.loginfo("TaskPlanner: origin set at odom (%.3f, %.3f) m",
                          px_m, py_m)

        dx_m = px_m - self.startup_origin_m[0]
        dy_m = py_m - self.startup_origin_m[1]
        self.current_pose_ft = (dx_m * FEET_PER_METER, dy_m * FEET_PER_METER)

    def _status_cb(self, msg):
        """
        ExecutionMonitor publishes:
          SUCCESS: reached target (within 1 ft)
          FAILURE: failed to reach target
        We use this to mutate remaining_sequence and possibly re-publish.
        """
        text = msg.data.strip()
        if not self.remaining_sequence:
            return

        if text.startswith("SUCCESS"):
            # we reached the current target => drop it
            self._pop_front()
            self.republished_after_failure = False

        elif text.startswith("FAILURE"):
            # drop current target in any case
            failed_elem = self._pop_front()

            # special rule: if we failed at a START,
            # remove that task's DEST from what's left
            if failed_elem and failed_elem["kind"] == "START":
                task_id = failed_elem["id"]
                before = len(self.remaining_sequence)
                self.remaining_sequence = [
                    e for e in self.remaining_sequence
                    if not (e["id"] == task_id and e["kind"] == "DEST")
                ]
                after = len(self.remaining_sequence)
                if before != after:
                    rospy.logwarn("TaskPlanner: removed DEST for task %s due to start failure.",
                                  task_id)

                # We changed the plan, so re-publish to nav
                self._publish_remaining_sequence()
                self.republished_after_failure = True

        # We ignore PROGRESS / STALLED for planning logic

    def _new_tasks_cb(self, msg):
        """
        Runtime re-tasking:
        Receive a brand new set of tasks as a multiline string, e.g.:

        data:
          "((2, 3), (9, 8))\n((12, 9), (4, 14))"

        Then we:
          - parse tasks
          - recompute best sequence
          - reset remaining_sequence
          - publish updated /task_planner/waypoints
        """
        text = msg.data.strip()
        if not text:
            rospy.logwarn("TaskPlanner: received empty new_tasks_text; ignoring.")
            return

        rospy.loginfo("TaskPlanner: received NEW TASK SET via /task_planner/new_tasks_text")
        self._plan_from_text_or_file(text_override=text, file_override=None)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _plan_from_text_or_file(self, text_override=None, file_override=None):
        """
        Helper that:
          1. loads tasks (either from provided string, or from file/param)
          2. waits for first odom if necessary
          3. computes new best_sequence
          4. resets remaining_sequence
          5. publishes waypoints for nav
        Returns True if a valid plan was published.
        """
        # 1. load tasks into self.tasks
        ok = self._load_tasks_dynamic(text_override, file_override)
        if not ok:
            return False

        # 2. make sure we have at least one odom reading so current_pose_ft isn't None
        #    (this also sets self.startup_origin_m)
        wait_count = 0
        while not rospy.is_shutdown() and self.current_pose_ft is None and wait_count < 200:
            rospy.sleep(0.05)
            wait_count += 1

        # 3. compute best order
        self.best_sequence = self._compute_best_sequence(self.tasks)
        self.remaining_sequence = list(self.best_sequence)
        rospy.loginfo("TaskPlanner: planned %d points.", len(self.best_sequence))

        # 4. publish
        self._publish_remaining_sequence()
        return True

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
        NavigationController's /task_planner/waypoints subscriber
        will parse this and start moving (or restart moving).
        """
        s = "; ".join(
            ["{:.3f},{:.3f}".format(e["pt"][0], e["pt"][1])
             for e in self.remaining_sequence]
        )
        self.waypoints_pub.publish(String(data=s))
        rospy.loginfo("TaskPlanner: published %d remaining waypoints.",
                      len(self.remaining_sequence))

    def _load_tasks_dynamic(self, text_override, file_override):
        """
        Parse tasks either from:
          - text_override (string passed in at runtime), OR
          - self.tasks_text_param (startup param), OR
          - file_override (path), OR
          - self.tasks_file (startup param)

        Produce self.tasks = [{"id": i, "start": (sx,sy), "dest": (dx,dy)}, ...]
        """
        # Pick source in priority order
        lines = []

        if text_override is not None and text_override.strip():
            raw_text = text_override.strip()
            lines = [ln for ln in raw_text.splitlines() if ln.strip()]

        elif self.tasks_text_param.strip():
            raw_text = self.tasks_text_param.strip()
            lines = [ln for ln in raw_text.splitlines() if ln.strip()]

        elif file_override is not None:
            try:
                with open(file_override, 'r') as f:
                    lines = [ln for ln in f.readlines() if ln.strip()]
            except Exception as e:
                rospy.logerr("TaskPlanner: failed to read override file: %s", e)
                return False

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
                rospy.logwarn("TaskPlanner: skipping bad line %d: %s",
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
            rospy.logwarn("TaskPlanner: no valid tasks parsed.")
            return False

        rospy.loginfo("TaskPlanner: loaded %d tasks.", len(tasks))
        return True

    # ------------------------------------------------------------------
    # Planner / Ordering
    # ------------------------------------------------------------------
    def _compute_best_sequence(self, tasks):
        """
        Build an order of START/DEST points such that:
          - each task's START is visited before that task's DEST
          - total straight-line travel distance is minimized
          - cost starts from the robot's provided starting pose
            (start_x_feet, start_y_feet)

        Output:
        [
          { "id": task_id,
            "kind": "START" or "DEST",
            "pt": (x,y) },
          ...
        ]
        """
        # Build flat list of (kind, id, point)
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
                for (k, t_id, _p) in partial:
                    if k == "START" and t_id == tid:
                        return True
                return False
            return True

        def path_cost(sequence):
            """
            Sum of Euclidean distances, starting from start_pose_feet.
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
            if not remaining:
                c = path_cost(partial)
                if c < best_cost_holder[0]:
                    best_cost_holder[0] = c
                    best_seq_holder[0] = list(partial)
                return

            # bias towards points closer to the robot's provided start pose
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

                if (best_seq_holder[0] is None or
                        path_cost(partial) <= best_cost_holder[0]):
                    backtrack(partial, next_remaining)

                partial.pop()

        backtrack([], candidates)

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
