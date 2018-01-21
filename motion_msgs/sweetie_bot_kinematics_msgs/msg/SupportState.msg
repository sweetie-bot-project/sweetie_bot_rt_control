# Robot weight distributon between legs.
#  * name --- the name of the chain.
#  * contact --- contact name 
#  * support --- leg support coefficients or z-component of reaction force. Support coefficient is the ratio of 
# the weight corresponding to the leg to the total weight of robot.
#
# All arrays sould have the same size or contain no elemnts.
#
Header header
string[] name
string[] contact
float64[] support
