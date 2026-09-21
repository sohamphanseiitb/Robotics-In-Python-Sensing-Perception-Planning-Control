"""
Scenario
At Petual, compliance platforms restrict how frequently users can request and download sensitive audit documents (like SOX financial records) to prevent bulk data exfiltration
. You are tasked with implementing a rate-limiting and document access tracking class in standard Python.
Requirements
Implement a class DocumentAccessTracker:
__init__(self, max_requests_per_min: int)
Initializes the tracker with a global per-user rate limit max_requests_per_min (e.g., maximum 5 granted requests allowed in any rolling 60-second window per user).
request_access(self, timestamp: int, user_id: str, doc_id: str) -> bool
timestamp is an integer in seconds (monotonically increasing globally).
Rate Limit Rule: Check how many successful requests user_id has made in the rolling 60-second window [timestamp - 60, timestamp].
If the count is >= max_requests_per_min, deny access: return False and do not record this denied request in their history.
Otherwise, grant access: record the event and return True.
get_user_accessed_docs(self, user_id: str) -> list[str]
Returns a list of unique doc_ids that user_id has successfully accessed across all time, ordered by when they were first accessed.
If the user has no successful accesses, return an empty list [].
get_flagged_documents(self, current_time: int, threshold: int) -> list[str]
Returns a list of doc_ids that have been successfully accessed by at least threshold distinct users in the last 300 seconds ([current_time - 300, current_time]).
Order of returned documents does not matter.
"""


"""
Thought Process:

1. Anchored entirely on the useer. Similar hasmpa like structure with user as the key. 
2. A hashmap: keys is user_id, and values or data as time, and doc_id
3. values will be implemented as a deque

request_access:
- index the docuemntTracker with user_id: 
- then find all access requests made in the past 60 seconds
- if acccess_requests > max_request_per_min: then deny and not record this

get_user_accessed_docs:
- index with user_id
- pick out unique document entries, and sort them by time of first access

get_flagged_documents:
- index with 

a hashmap indexed on user_id: [timetsmap1, doc_id1], [timestamp2, doc-id2] amd sp on
another hasmpa indeces on doc_id: [timetsmap1, user_id1], []
"""

from collections import defaultdict, deque

class DocumentAccessTracker:

    def __init__(self, max_requests_per_min: int):
        self.mReqs = max_requests_per_min

        # user_id is the anchor
        self.userLogs = defaultdict(deque)

        # doc_id is the anchor
        self.docLogs = defaultdict(deque)

        # creating 2 diff hashmaps is comprosmising on memory but then time parsing will be much faster in the latter 2 fucntions

    def request_access(self, timestamp: int, user_id: str, doc_id: str)-> bool:

        user_history = self.userLogs[user_id]
        doc_history = self.docLogs[doc_id]

        # check how many successful attempts they made in the last 60 seconds
        # we only record successful actions: so summing all timestamps which satisfy the 60 second threshold is good enough
        ## no need to check actions
        numAttempts = sum(1 for t, _ in user_history if t >= timestamp - 60)
        if numAttempts >= self.mReqs:
            # don't record denied requests: niether in userLogs, nither in docLogs
            return False
        else:
            # grant access: record in both userLogs and docLogs
            user_history.append((timestamp, doc_id))
            doc_history.append((timestamp, user_id))
            return True

    def get_user_accessed_docs(self, user_id: str) -> list[str]:

        # get access to user history
        user_history = self.userLogs[user_id]

        # find all the unique documents accessed by them, and order them according to time of first access
        # allDocs = {docID for t, docID in user_history} # set comprehension
        # not sure if this squashes the order in case a document was requested multiple times, there will be mulitple entrues of the same doc_id

        # Preserves order of FIRST access while removing duplicates
        allDocs = dict.fromkeys(doc_id for _, doc_id in user_history)

        return list(allDocs)

    def get_flagged_documents(self, current_time: int, threshold: int) -> list[str]:

        # find out all possible document IDs
        allDocIds = self.docLogs.keys()

        # doc IDs which have >= threshold number of distinct users
        docflagged = []

        # iterate over all docIDs
        for docID in allDocIds:

            # access that document histpry
            doc_history = self.docLogs[docID]

            # first only find user IDs for all granted requests based on timestamp criterion and then do unique users using set comprehension
            allReqsFordocID = {userID for t, userID in doc_history if t>= current_time-300}

            # find if the condition is satisfied
            if len(allReqsFordocID) >= threshold:
                docflagged.append(docID)

        return docflagged



