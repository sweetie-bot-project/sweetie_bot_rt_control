# Acknowledge of ResourceAssigment (is_operational = true) or notification about deactivation (is_operational = false).
string requester_name 
uint8 state
# state values
uint8 NONOPERATIONAL = 0
uint8 PENDING = 1
uint8 OPERATIONAL = 2
