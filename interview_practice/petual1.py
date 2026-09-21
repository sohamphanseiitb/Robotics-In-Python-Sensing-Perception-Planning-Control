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
import time
from typing import tuple
import numpy as np

class AuditLogMonitor:

    def __init__(self):
        self.auditLog = {}

    def record_action(self, timestamp: int, user_id: str, action_type: str) -> str:

        # check if user is a first time user
        if user_id is not in self.auditLog.keys():
            self.auditLog[user_id] = []

        # otherwise
        else:
            self.auditLog[user_id].append(tuple(timestamp, action_type))
        
        # Rule 1:
        self.r1Times = np.where(timestamp - 10 < self.auditLog[user_id][:, 0] and self.auditLog[user_id][:, 0] < timestamp)
        if len(self.r1Times) >= 5:
            return "VELOCITY_STRIKE"

        # Rule 2:
        """
           2. all rows indices between timestamp-300 adn timestamp: indicesnp.where(timestamp -300 < auditLog[userID][:, 1] < timestamp)
               3. all actions between timestamp-300 and timestamp: allActions = auditLog[userID][indices]
               4. If 'EXPORT_DATA' in allActions? (check if 'LOGIN'? do nothing: publish "UNAUTHE_EXPORT") : "do nothing" 
            """
        self.r2Times = np.where(timestamp - 300 < self.auditLog[user_id][:, 0] and self.auditLog[user_id][:, 0] < timestamp)
        self.r2Actions = self.auditLog[user_id][self.r2Times]
        if "EXPORT_DATA" in self.r2Actions and "LOGIN" not in self.r2Actions:
            return "UNAUT_EXPORT"

    def get_top_active_user(self, current_time: int)-> str:
        """
        - for user find: num(indices):= indices = np.where(timestamp-60 < auditLog[userID][:, 0] < timestamp)
        - user_actopns: key: useriD, value: num(indices)
        - find max_by_value: return index of argmax
        """
        self.allUsers = self.auditLog.keys()
        self.topUser = ""
        self.maxActions = 0
        for every_user in self.allUsers:
            self.topUserIdxs = np.where(current_time - 60 < self.auditLog[every_user][:, 0] and self.auditLog[every_user][:, 0] < current_time)
            if len(self.topUserIdxs) > self.maxActions:
                self.topUser = every_user
                self.maxActions = len(self.topUserIdxs)

        return self.topUser







