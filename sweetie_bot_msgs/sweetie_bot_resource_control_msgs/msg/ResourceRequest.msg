# name of the requester
string requester_name

# names of the requested resources
string[] resources


# priorities of giving the corresponding resource (pending) to this requester
# and holding it after acquiring (operational)
# range is [0:1]
float32[] pri_pending
float32[] pri_operational
