#!/usr/bin/env python3

from __future__ import print_function
import rospy
from gazebo_msgs.msg import ModelState
from std_msgs.msg import Int32MultiArray

class SpawnPosition:
    """
    @class SpawnPosition
    @brief Wrapper class for respawning robots anywhere.

    Listens for messages on dedicated topic for teleport positions, then spawns robot in requested position.
    """

    def spawn_position(self, position):
        msg = ModelState()
        msg.model_name = 'B1'

        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = position[3]
        msg.pose.orientation.y = position[4]
        msg.pose.orientation.z = position[5]
        msg.pose.orientation.w = position[6]

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', msg)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")

    def __init__(self):
        position_topic = rospy.get_param("~position_topic", "/spawn_position")
        self.pos_sub = rospy.Subscriber(position_topic, Int32MultiArray, self.spawn_position, queue_size=1)

def main():
    rospy.init_node('spawn_position', anonymous=True)
    ic = SpawnPosition()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
