#! /usr/bin/env python
import rospy
import tf

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('ee_pose_demo')
    wait_for_time()

    listener = tf.TransformListener()
    rospy.sleep(0.5) #Lab23 said only 0.1 was necessarry but I'm just playing it safe

    while not rospy.is_shutdown():
        print(f"Time: {rospy.Time.now().to_sec()}")
        try:
            #                                                   end effector     base        get most recent        
            translation, quaternion = listener.lookupTransform("gripper_link", "base_link", rospy.Time(0))
            #print("Translation: ", translation)
            #print("Quartenion: ", quartenion)
            print(F"{translation} {quaternion}")
        except tf.ConnectivityException:
            print("Exception: ConnectivityException")
        except tf.LookupException:
            print("Exception: LookupException")
        except tf.ExtrapolationException:
            print("Exception: ExtrapolationException")
        print("\n")


        # Sleep for 1s
        rospy.sleep(rospy.Duration(1.0))




if __name__ == '__main__':
    main()
