#!/bin/sh
VARIABLE="
{
 header: auto,
 name: ['joint11', 'joint12', 'joint13', 'joint14', 'joint15', 'joint16',
        'joint21', 'joint22', 'joint23', 'joint24', 'joint25', 'joint26',
        'joint31', 'joint32', 'joint33', 'joint34', 'joint35', 'joint36',
        'joint41', 'joint42', 'joint43', 'joint44', 'joint45', 'joint46',
        'joint51', 'joint52', 'joint53', 'joint54',
        'joint55', 'joint56',
        'joint_eyes_focus_x', 'joint_eyes_focus_y', 'joint_eyes_focus_z'
       ],
 position: [0.11, 0.12, 0.13, 0.14, 0.15, 0.16,
            0.21, 0.22, 0.23, 0.24, 0.25, 0.26,
            0.31, 0.32, 0.33, 0.34, 0.35, 0.36,
            0.41, 0.42, 0.43, 0.44, 0.45, 0.46,
            0.51, 0.52, 0.53, 0.54,
            0.55, 0.56,
            0.61, 0.62, 0.63
           ],
 velocity: [],
 effort:   []
}"

echo "<<<" "$VARIABLE" "\n<<<"
echo ">>>"
rostopic pub /kinematics/input_joint_seed sensor_msgs/JointState --once "$VARIABLE"
echo ">>>"
