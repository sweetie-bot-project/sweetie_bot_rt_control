# Controllers state message published by ResourceArbiter on any controller state change. All arrays have the same length.
# * name --- list of controllers.
# * state --- controler states.
# * request_id --- ID of last message exchange with controller.
# request_id increases with each resource request. It must be equal to or greater then request_id from ResourceRequest message
# if it was processed by arbiter and confirmation form controller is received.
string[] name
uint32[] request_id
uint8[] state
# state constants
uint8 NONOPERATIONAL=0
uint8 PENDING=1
uint8 OPERATIONAL=2
uint8 OPERATIONAL_PENDING=3
