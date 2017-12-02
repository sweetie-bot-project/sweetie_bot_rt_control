# Resource assignment request.
#  * requester_name --- name of the controller component emitting request.
#  * request_id --- high 16 bits contains controller request counter number, low 16 bits contains an unique requester indentifer.
#  * resources --- resources set (kinematic chains names: leg1, leg2, ..., tail, eyes).
#  * pending_pri --- priorities in pending state (same size as resources).
#  * operational_pri --- priorities in operational state (same size as resources).
#
# request_id is optional if operation is used to emmit request.
#

string requester_name 
uint32 request_id
string[] resources
float32[] pri_operational
float32[] pri_pending
