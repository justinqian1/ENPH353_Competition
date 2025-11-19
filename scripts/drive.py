#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String
from enum import Enum

class States(Enum):
    FWD=0
    LEFT=1
    RIGHT=2
    STOP_PED=3
    STOP_TRUCK=4
    STOP_MAXTIME=5

start_timer = String('team,pass,0,whatever')
stop_timer = String('team,pass,-1,whatever')

class Driver:
    def __init__(self):
        self.data = rospy.Subscriber('/drive_info',Int32MultiArray, callback=self.callback,queue_size=1)
        self.drive_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.state = States.FWD 
        self.forward_speed = 0.5
        self.turn_speed = 1.0
        self.time_pub.publish(start_timer)
        #self.start_time = rospy.Time.now()
        #self.duration = rospy.Duration(120.0) # drive for 10 s

    def callback(self, msg):
        #[line_in_front,line_left_coord,line_right_coord,red_line_sz,pink_line_sz,ped_sz,truck_sz]
        line_fwd,line_L,line_R,red_ln,pink_ln,ped,truck=msg.data
        if self.state == States.FWD:
            if line_fwd>10:
                self.state=States.RIGHT if line_L > line_R else States.LEFT
            if red_ln>10_000:
                self.state=States.STOP_PED
                self.drive_pub.publish(Twist())
                self.time_pub.publish(stop_timer)
                return
        elif (self.state == States.LEFT and line_L > line_R) or \
           (self.state == States.RIGHT and line_R > line_L):
            self.state = States.FWD
        '''
        elapsed = rospy.Time.now() - self.start_time
        if elapsed > self.duration:
            self.state=States.STOP_MAXTIME
            self.drive_pub.publish(Twist())
            self.time_pub.publish(stop_timer)
            return
        '''
        print(self.state)
        twist = Twist()
        if self.state==States.FWD:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0
        elif self.state==States.LEFT:
            twist.linear.x = 0
            twist.angular.z = self.turn_speed
        elif self.state==States.RIGHT:
            twist.linear.x = 0
            twist.angular.z = -self.turn_speed
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        self.drive_pub.publish(twist)

def main():
    rospy.init_node('driver')
    d = Driver()
    rospy.sleep(0.2) 
    d.time_pub.publish(start_timer)
    rospy.spin()
    

if __name__ == '__main__':
    main()