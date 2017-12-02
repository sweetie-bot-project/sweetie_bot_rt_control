#!/bin/sh
VARIABLE="
{
 header: auto,
 name: ['joint51', 'joint52', 'joint53', 'joint54', 'joint15',
        'joint21', 'joint22', 'joint23', 'joint24', 'joint25',
        'joint31', 'joint32', 'joint33', 'joint34', 'joint35',
        'joint41', 'joint42', 'joint43', 'joint44', 'joint45',
        'joint11', 'joint12', 'joint13', 'joint14'],
 position: [0.2, 0.3, 0.4, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0],
 velocity: [],
 effort:   []
}"
echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /agregator/input_joint_state sensor_msgs/JointState --once "$VARIABLE"
echo ">>>"
