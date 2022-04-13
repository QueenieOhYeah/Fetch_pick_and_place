#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetTorso, SetTorsoResponse, SetHead, SetHeadResponse, SetArm, SetArmResponse, SetGripper, SetGripperResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._arm = robot_api.Arm()
        self._gripper = robot_api.Gripper()

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        h = request.height
        self._torso.set_height(h)
        return SetTorsoResponse()
        
    def handle_set_head(self, request):
        # TODO: move the torso to the requested height
        p = request.pan
        t = request.tilt
        self._head.pan_tilt(p, t)
        return SetHeadResponse()
        
    def handle_set_arm(self, request):
        # TODO: move the torso to the requested height
        arm_vals = [request.shoulder_pan, request.shoulder_lift, request.upperarm_roll, request.elbow_flex, request.forearm_roll, request.wrist_flex, request.wrist_roll]
        self._arm.move_to_joints(robot_api.ArmJoints.from_list(arm_vals))

        return SetArmResponse()
        
    def handle_set_gripper(self, request):
        # TODO: move the torso to the requested height
        if request.effort == -1:
            self._gripper.open()
            return SetGripperResponse()  
        self._gripper.close(request.effort)
        return SetGripperResponse()       


def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead,
                                  server.handle_set_head)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm,
                                  server.handle_set_arm)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper,
                                  server.handle_set_gripper)                    
    rospy.spin()


if __name__ == '__main__':
    main()
