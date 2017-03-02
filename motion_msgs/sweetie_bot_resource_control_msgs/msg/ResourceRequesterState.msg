# Acknowledge of ResourceAssigment (is_operational = true) or notification about deactivation (is_operational = false).
#  * requester_name --- name of the controller component emitting request.
#  * request_id / request_number --- the indentifer of last ResourceRequester emitted by controller, 
#       or resource arbiter request counter value from ResourceAssigment message.
#  * is_operational --- flag operational state.
#
string requester_name 
uint32 request_id
bool is_operational
