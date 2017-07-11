# Servo model parameters.
#
# (g/kgear-q)*battery_voltage*kp = alpha[0]*dq + alpha[1]*d2q + alpha[2]*sign(dq) + alpha[3]*sign(dq)*exp(-|dq/qs|^delta) + alpha[4]*tau 
# 
#  * name --- servo identification string.
#  * kp --- proportional coefficient of servo P-regulator.
#  * kgear --- ratio of reduction gear (number of turns of servo per one turn of joint)
#  * alpha[5] --- servo model and friction parameters (identifcation result).
#  * qs, delta --- Stribeck parameteres.
#
string name
float64 kp
float64 kgear
float64[5] alpha
float64 qs
float64 delta
