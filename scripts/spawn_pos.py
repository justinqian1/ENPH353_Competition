#!/usr/bin/env python3

from __future__ import print_function
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R

class SpawnPosition:
    """
    @class SpawnPosition
    @brief Wrapper class for respawning robots anywhere.

    Listens for messages on dedicated topic for teleport positions, then spawns robot in requested position.
    """

    def __init__(self):
        position_topic = rospy.get_param("~position_topic", "/spawn_position")
        self.pos_sub = rospy.Subscriber('/spawn_position', Float32MultiArray, self.callback, queue_size=10)

    def callback(self, data):
        position = self.eul_to_qua(data.data)
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
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( msg )

        except rospy.ServiceException:
            print ("Service call failed")

    def eul_to_qua(self, eul_rep):
        quat = R.from_euler('xyz', eul_rep[-3:], degrees=False).as_quat()
        return list(eul_rep[:3]) + quat.tolist()

def main():
    rospy.init_node('spawn_position', anonymous=True)
    ic = SpawnPosition()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
