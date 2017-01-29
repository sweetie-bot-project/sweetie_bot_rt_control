# Resource assignment to controllers.
#  * request_ids / request_number --- the list of indentifers of last ResourceRequests of pending controllers
#       or resource arbiter request counter value from ResourceAssigment message.
#  * resource --- assignment resources list.
#  * owner --- owner controller name.
uint32[] request_ids 
string[] resources
string[] owners
