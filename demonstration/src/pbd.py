#!/usr/bin/env python

import rospy
import pickle
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from demonstration.msg import UserAction
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from robot_controllers_msgs.msg import ControllerState, QueryControllerStatesGoal, QueryControllerStatesAction
from nav_msgs.msg import Odometry
from tf import TransformListener
import actionlib
from robot_api import Arm, Gripper


DEBUG = True

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


#subscribe to useraction
class PbD_Create_Pose_Server(object):
    def __init__(self):
        self.pickle_file = "PbDpositions.p"
        # List of detected markers
        self.markers = []
        self.poses = []

        # 
        self._arm = Arm()
        def shutdown():
            self._arm.cancel_all_goals()
        rospy.on_shutdown(shutdown)

        self._gripper = Gripper()
        self._gripper_status = "open" #TODO: actually check initial gripper status

        self._listener = TransformListener()

        #self.dict = self.unpickling(self.pickle_file)
        #if self.dict == None:
        #  self.dict = {}
        
        # Subscribe to AR tag poses, use reader.callback
        # To detect markers:
        # Need to: roslaunch robot_api ar_desktop.launch cam_image_topic:=<cloud point topic name>
        self.marker_sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.marker_callback) 

        self.user_action = rospy.Subscriber('demonstration/user_actions', UserAction, self.callback_user_action)

        #self._controller_client = actionlib.SimpleActionClient('arm_relaxer', QueryControllerStatesAction)
        self._controller_client = actionlib.SimpleActionClient('query_controller_states', QueryControllerStatesAction)
        print("waiting for controller server")
        self._controller_client.wait_for_server()
        print("waited")

    def marker_callback(self, msg):
        self.markers = msg.markers


    """
    def unpickling(self, filename):
        #filename = "fetch_position"
        try:   
            with open(filename + '.pkl', 'rb') as infile:
                obj = pickle.load(infile)
                #infile.close()
                return obj #TODO: make this a callback instead
        except:
            return None
    """
            
    # Helper function, might not needed
    #def pose_to_stampedpose(self, pose: Pose):
    #    stamped_pose = PoseStamped()
    #    stamped_pose.pose = pose
    #    return stamped_pose

    def callback_user_action(self, data: UserAction):
        print(f"CALLBACK DATA IS: {data}")

        if data.command == UserAction.RELAX:
            # create the program, relax arm
            print("program started. relaxing arm.")
            self._relax_arm()

            if data.filename != '':
                self.poses = [] # clear any poses in case we've created another program or loaded one in the past
                self.pickle_file = data.filename + ".p"
            

        if data.command == UserAction.SAVE_TO_BASE:
            self.save_to_base()

        if data.command == UserAction.SAVE_TO_TAG:
            # transform pose and save poses related to tag
            # ...
            #self.save_poses(pose, gripper)
            pass

        if data.command == UserAction.OPEN:
            # open gripper
            print("open start")
            self._start_arm()
            print("opening gripper")
            self._gripper.open()
            self._gripper_status = "open"
            self.save_to_base() #TODO: call whichever save function was used previously
            self._relax_arm()
            
        if data.command == UserAction.CLOSE:
            # close gripper
            print("close start")
            self._start_arm()
            print("closing gripper")
            self._gripper.close()
            self._gripper_status = "close"
            self.save_to_base() #TODO: call whichever save function was used previously
            self._relax_arm()

        if data.command == UserAction.CREATE:
            # save program
            #self.pickling()
            # data structure for saving can be [[pose1, relation1, gripper_state],[pose2, relation2, gripper2], [pose3, relation3, gripper3]]
            #def pickling(self, d):
            print(f"pickling {self.pickle_file}")
            print("poses are")
            print(self.poses)
            #with open(self.pickle_file + '.pkl', 'wb') as outfile:
            #    pickle.dump(self.poses, outfile)
            pickle.dump(self.poses, open(self.pickle_file, "wb"))
            #outfile.close()
            print("pickled")

        if data.command == UserAction.RUN:
            self.pickle_file = data.filename + ".p"
            print(f"Opening {self.pickle_file}")  

            try:
                self.poses = pickle.load(open(self.pickle_file, "rb"))
            except:
                print("Error reading pickle file")

            # TODO: handle gripper state and base vs. tag relation
            for pose_packet in self.poses:
                ps = pose_packet[0]
                #relation = pose_packet[1]
                gripper_status = pose_packet[1]

                if gripper_status != self._gripper_status:
                    rospy.sleep(0.3)
                    
                    if gripper_status == 'close':
                        self._gripper.close()
                    elif gripper_status == 'open':
                        self._gripper.open()
                    self._gripper_status = gripper_status
                    rospy.sleep(3)
                    
                else:
                    self._arm.move_to_pose(ps)
                #rospy.sleep(3)
                print("\n\n")

    def save_to_base(self):
        #                                                         end effector     base        get most recent        
        #translation, quaternion = self._listener.lookupTransform("gripper_link", "base_link", rospy.Time(0))
        translation, quaternion = self._listener.lookupTransform("base_link", "wrist_roll_link", rospy.Time(0))
            
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose.position.x = translation[0]
        ps.pose.position.y = translation[1]
        ps.pose.position.z = translation[2]
        ps.pose.orientation.x = quaternion[0]
        ps.pose.orientation.y = quaternion[1]
        ps.pose.orientation.z = quaternion[2]
        ps.pose.orientation.w = quaternion[3]

        self.save_poses(ps, self._gripper_status)
            
    def save_poses(self, pose: PoseStamped, gripper_status):
        self.poses.append([pose, gripper_status])
    
    '''Get pose for each marker usage:
       for marker in self.markers: 
           get_marker_pose(marker) '''    
    def get_marker_pose(self, marker:AlvarMarker):
        # usage: position.x, position.y, position.z
        position = marker.pose.pose.position        
        quaternion = marker.pose.pose.orientation
        return position, quaternion
    
    def _relax_arm(self):
        self._change_arm_state(ControllerState.STOPPED)

    def _start_arm(self):
        self._change_arm_state(ControllerState.RUNNING)

    def _change_arm_state(self, new_state):
        try:
            if rospy.get_param("use_sim_time"):
                print("arm can only be turned on and off on real robot")
                return
        except:
            pass

        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = new_state
        goal.updates.append(state)
        self._controller_client.send_goal(goal)
        print("updating arm state")
        self._controller_client.wait_for_result()
        print(f"updated to: {new_state}")


def main():
    print("starting server")
    rospy.init_node('create_pose_server')
    server = PbD_Create_Pose_Server()
    wait_for_time()
    print("server started")
    rospy.spin()


if __name__ == '__main__':
    main()
