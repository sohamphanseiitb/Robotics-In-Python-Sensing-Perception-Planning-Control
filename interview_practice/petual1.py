"""
🚨 Mock Interview Problem: Audit Log Anomaly Monitor
Scenario
At Petual, compliance engines process streaming audit logs (such as system access events, document views, or financial overrides) to detect anomalies in real-time
. You are tasked with implementing a stateful monitoring class that tracks user activity and flags potential security or compliance violations.
Requirements
Implement a class AuditLogMonitor:
__init__()
Initialize any internal data structures needed to store user activity efficiently.
record_action(timestamp: int, user_id: str, action_type: str) -> list[str]
Records a user event, where timestamp is in seconds (monotonically increasing globally).
Evaluates and returns a list of trigger messages if any of the following rules are violated by this action:
Rule 1 (Velocity Spike): The user has performed more than 5 total actions within the last 10 seconds (inclusive: [timestamp - 10, timestamp]).
Trigger message: "VELOCITY_SPIKE"
Rule 2 (Unauthenticated Export): An "EXPORT_DATA" action occurs without a preceding "LOGIN" action by that same user within the last 300 seconds (inclusive: [timestamp - 300, timestamp]).
Trigger message: "UNAUTHENTICATED_EXPORT"
get_top_active_user(current_time: int) -> str | None
Returns the user_id with the highest number of recorded actions in the last 60 seconds ([current_time - 60, current_time]).
If no actions exist in that window, return None. (Tie-breaking: return any of the top users).

"""

"""
class AuditLogMonitor

1. All the actions are anchrods on the user. Unauthenticated export of files, user taking action, uise logging in et6c
2. Lets create a hashmap, where the key is the user_id, and then the value is a list of tuples which contain (timestamp, action)
3. we are going to need a timestamp tracker, keep incrementing the time
4. Meed a function/logic whioch computes how many actions a user has committed between (t0, t1)
4. Implement Rule 1 and Rule 2:
5. Rule 1:
    1. all timestamps: number(timestamp-10 < auditLog[userID][:, 0] < timestamp) >= 5? "VELOCITY_STRIKE": nothing
6. Rule 2: 
    2. all rows indices between timestamp-300 adn timestamp: indicesnp.where(timestamp -300 < auditLog[userID][:, 1] < timestamp)
    3. all actions between timestamp-300 and timestamp: allActions = auditLog[userID][indices]
    4. If 'EXPORT_DATA' in allActions? (check if 'LOGIN'? do nothing: publish "UNAUTHE_EXPORT") : "do nothing"
7.  get top active user with hjighest actions in the previous 60 seconds:
    - for user find: num(indices):= indices = np.where(timestamp-60 < auditLog[userID][:, 0] < timestamp)
    - user_actopns: key: useriD, value: num(indices)
    - find max_by_value: return index of argmax
"""
from collections import defaultdict, deque

class AuditLogMonitor:

    def __init__(self):
        # Maps user_id -> deque of tuples: (timestamp, action_type)
        self.user_history = defaultdict(deque)

    def record_action(self, timestamp: int, user_id: str, action_type: str) -> list[str]:
        triggers = []
        user_events = self.user_history[user_id]
        
        # 1. Append current event
        user_events.append((timestamp, action_type))

        # 2. Check Rule 1: Velocity Spike (> 5 actions in last 10s: [timestamp - 10, timestamp])
        cutoff_r1 = timestamp - 10
        # Count events in user_events where event_time >= cutoff_r1
        actions_in_10s = sum(1 for t, _ in user_events if t >= cutoff_r1)
        if actions_in_10s > 5:
            triggers.append("VELOCITY_SPIKE")

        # 3. Check Rule 2: Unauthenticated Export
        # Triggers ONLY if current action is EXPORT_DATA and NO "LOGIN" occurred in [timestamp - 300, timestamp]
        if action_type == "EXPORT_DATA":
            cutoff_r2 = timestamp - 300
            has_login = any(act == "LOGIN" for t, act in user_events if t >= cutoff_r2)
            if not has_login:
                triggers.append("UNAUTHENTICATED_EXPORT")

        return triggers

    def get_top_active_user(self, current_time: int) -> str | None:
        top_user = None
        max_actions = 0
        cutoff_60s = current_time - 60

        for user_id, user_events in self.user_history.items():
            # Count actions in the last 60 seconds
            recent_count = sum(1 for t, _ in user_events if t >= cutoff_60s)
            if recent_count > max_actions:
                max_actions = recent_count
                top_user = user_id

        return top_user if max_actions > 0 else None






