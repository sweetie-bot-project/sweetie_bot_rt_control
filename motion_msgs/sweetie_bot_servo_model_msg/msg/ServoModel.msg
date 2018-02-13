# Servo model parameters.
#
# (g/kgear-q)*battery_voltage*kp = alpha[0]*dq + alpha[1]*sign(dq) + alpha[2]*sign(dq)*exp(-|dq/qs|^delta) + alpha[3]*tau 
# 
#  * name --- servo identification string.
#  * kp --- proportional coefficient of servo P-regulator.
#  * kgear --- ratio of reduction gear (number of turns of servo per one turn of joint)
#  * alpha[4] --- servo model and friction parameters (identifcation result).
#  * qs, delta --- Stribeck parameteres.
#
string name
float64 kp
float64 kgear
float64[4] alpha
float64 qs
float64 delta
