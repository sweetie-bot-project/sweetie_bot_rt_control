#!/bin/sh
VARIABLE="
{
 header: auto,
 name: ['joint11', 'joint12', 'joint13', 'joint14', 'joint15',
        'joint21', 'joint22', 'joint23', 'joint24', 'joint25',
        'joint31', 'joint32', 'joint33', 'joint34', 'joint35',
        'joint41', 'joint42', 'joint43', 'joint44', 'joint45',
        'joint51', 'joint52', 'joint53', 'joint54'],
 position: [1.11, 1.12, 1.13, 1.14, 1.15,
            1.21, 1.22, 1.23, 1.24, 1.25,
            1.31, 1.32, 1.33, 1.34, 1.35,
            1.41, 1.42, 1.43, 1.44, 1.45,
            1.51, 1.52, 1.53, 1.54],
 velocity: [2.11, 2.12, 2.13, 2.14, 2.15,
            2.21, 2.22, 2.23, 2.24, 2.25,
            2.31, 2.32, 2.33, 2.34, 2.35,
            2.41, 2.42, 2.43, 2.44, 2.45,
            2.51, 2.52, 2.53, 2.54],
 effort:   [3.11, 3.12, 3.13, 3.14, 3.15,
            3.21, 3.22, 3.23, 3.24, 3.25,
            3.31, 3.32, 3.33, 3.34, 3.35,
            3.41, 3.42, 3.43, 3.44, 3.45,
            3.51, 3.52, 3.53, 3.54]

}"
echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /joint_state sensor_msgs/JointState --once "$VARIABLE"
echo ">>>"
