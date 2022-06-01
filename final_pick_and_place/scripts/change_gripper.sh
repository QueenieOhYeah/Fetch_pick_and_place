#!/bin/bash
FD=$(rospack find fetch_description)
DIR="${FD}/meshes"
L_EXT_FILE="${DIR}/l_gripper_finger_link_extensions.STL"
R_EXT_FILE="${DIR}/r_gripper_finger_link_extensions.STL"
L_STANDARD_FILE="${DIR}/l_gripper_finger_link_standard.STL"
R_STANDARD_FILE="${DIR}/r_gripper_finger_link_standard.STL"

if [ $1 = "extensions" ]; then
	echo "Setting gripper to use modified extensions"
	cp $L_EXT_FILE $DIR/l_gripper_finger_link.STL
	cp $R_EXT_FILE $DIR/r_gripper_finger_link.STL
elif [ $1 = "standard" ]; then
	echo "Setting gripper to use standard fingers"
	cp $L_STANDARD_FILE $DIR/l_gripper_finger_link.STL
	cp $R_STANDARD_FILE $DIR/r_gripper_finger_link.STL
else
	echo "Please specify 'standard' or 'extensions' as your gripper choice."
fi

