#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String
from enum import Enum, auto

class States(Enum):
    FWD=auto()
    FWD_LOCK=auto()
    LEFT=auto()
    LEFT_LOCK=auto()
    RIGHT=auto()
    FWD_LEFT=auto()
    FWD_LEFT_LOCK=auto()
    FWD_RIGHT=auto()
    ALIGN_LEFT=auto()
    ALIGN_RIGHT=auto()
    STOP_TEMP=auto() # only for transitioning bw fwd/turn
    STOP_PED_NOTSEEN=auto()
    STOP_PED_SEEN=auto()
    STOP_TRUCK=auto()
    STOP_MAXTIME=auto()

start_timer = String('team,pass,0,whatever')
stop_timer = String('team,pass,-1,whatever')

LINE_FWD_BOX_TH=5 # num px in driving box
LINE_EDGE_TOL=18 # diff bw lineL and lineR to go back to moving fwd
LINE_MID_DIFF_TH=10 # threshold for line middle vs side (to decide to go fwd+left/fwd+right)
LINE_M_TH=190 # y threshold for line middle to matter
LINE_M_LOOP_TH=200 # line middle in the loop (i.e. begin hardcode truck portion)
LINE_LR_LOOP_TH=200 # left and right thresholds for loop
RED_LN_TH=2000 # num px of red line needed
PED_TH=10 # num px of ped to count as seen
ROAD_L_TH=180 # road y coord to exit line
XWALK_LOCK_TIME=1.2 # time to lock fwd state in crosswalk
LEFT_LOCK_TIME=0.8 # lock left turn at start of loop
EXIT_LOOP_WAIT_TIME=4.0 # wait time before logic to exit loop hits
EXIT_LOOP_LOCK_TIME=1.0 # lock time for fwd left while exiting

class Driver:
    def __init__(self):
        self.data = rospy.Subscriber('/drive_info',Int32MultiArray, callback=self.callback,queue_size=1)
        self.drive_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.state = States.FWD 
        self.exit_loop_time=rospy.Time.now() + rospy.Duration(10000) # time before we MIGHT consider exiting
        self.past_ped=False
        self.past_loop=False
        self.forward_speed = 1.8
        self.turn_speed = 5.0

        self.time_pub.publish(start_timer)
        #self.start_time = rospy.Time.now()
        #self.duration = rospy.Duration(120.0) # drive for 10 s

    def callback(self, msg):
        line_fwd,line_L,line_M,line_R,red_ln,pink_ln,ped,truck,road_L=msg.data
        now=rospy.Time.now()
        if self.state in [States.FWD,States.FWD_LEFT,States.FWD_RIGHT]:
            if red_ln>RED_LN_TH and not self.past_ped: # STOP FOR PED
                if line_L > line_R+LINE_EDGE_TOL:
                    self.state=States.ALIGN_RIGHT
                elif line_R > line_L+LINE_EDGE_TOL:
                    self.state=States.ALIGN_LEFT
                else:
                    self.state=States.STOP_PED_NOTSEEN if ped<PED_TH else States.STOP_PED_SEEN
                #self.drive_pub.publish(Twist())
                #self.time_pub.publish(stop_timer)
                #return
            elif line_M>LINE_M_LOOP_TH and line_L>LINE_LR_LOOP_TH and line_R>LINE_LR_LOOP_TH: # ENTERING LOOP
                self.state=States.LEFT_LOCK
                self.exit_loop_time=now+rospy.Duration(EXIT_LOOP_WAIT_TIME)
                self.locked_until = now + rospy.Duration(LEFT_LOCK_TIME)
            elif road_L< ROAD_L_TH and not self.past_loop and now > self.exit_loop_time:
                self.state=States.FWD_LEFT_LOCK
                self.past_loop=True
                self.locked_until = now + rospy.Duration(EXIT_LOOP_LOCK_TIME)
            elif line_fwd>LINE_FWD_BOX_TH: # TURN B/C WE'RE DRIVING AT THE LINE
                self.state=States.STOP_TEMP
            else: # logic to change states
                if self.state == States.FWD:
                    if line_M>LINE_M_TH and line_M < line_L-LINE_MID_DIFF_TH:
                        self.state=States.FWD_RIGHT
                    elif line_M > LINE_M_TH and line_M < line_R-LINE_MID_DIFF_TH:
                        self.state=States.FWD_LEFT
                if (self.state == States.FWD_LEFT and line_L > line_R-LINE_EDGE_TOL) or \
                (self.state == States.FWD_RIGHT and line_R > line_L-LINE_EDGE_TOL):
                    self.state=States.FWD
        
                
        elif (self.state == States.LEFT and line_L > line_R-LINE_EDGE_TOL) or \
             (self.state == States.RIGHT and line_R > line_L-LINE_EDGE_TOL):
            self.state = States.STOP_TEMP
        
        #align to red line
        elif (self.state == States.ALIGN_LEFT and line_L > line_R-LINE_EDGE_TOL) or \
             (self.state == States.ALIGN_RIGHT and line_R > line_L-LINE_EDGE_TOL):
            self.state=States.STOP_PED_NOTSEEN if ped<PED_TH else States.STOP_PED_SEEN

        elif self.state==States.STOP_TEMP:
            if line_fwd>LINE_FWD_BOX_TH: # fwd -> turn
                self.state=States.RIGHT if line_L > line_R else States.LEFT
            else: # turn -> fwd
                self.state=States.FWD
        elif self.state==States.STOP_PED_NOTSEEN and ped > PED_TH: # ped not yet crossed -> ped crossing
            self.state=States.STOP_PED_SEEN
        elif self.state==States.STOP_PED_SEEN and ped < PED_TH: # ped crossing -> ped done crossing
            self.state=States.FWD_LOCK
            self.past_ped=True
            self.locked_until = rospy.Time.now() + rospy.Duration(XWALK_LOCK_TIME)
        elif self.state in [States.FWD_LOCK,States.LEFT_LOCK,States.FWD_LEFT_LOCK] and now > self.locked_until:
            self.state=States.STOP_TEMP

        '''
        elapsed = rospy.Time.now() - self.start_time
        if elapsed > self.duration:
            self.state=States.STOP_MAXTIME
            self.drive_pub.publish(Twist())
            self.time_pub.publish(stop_timer)
            return
        '''

        twist = Twist()
        if self.state in [States.FWD, States.FWD_LOCK]:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0
        elif self.state==States.FWD_LEFT:
            twist.linear.x = self.forward_speed
            twist.angular.z = self.turn_speed*0.6
        elif self.state==States.FWD_LEFT_LOCK:
            twist.linear.x = self.forward_speed
            twist.angular.z = self.turn_speed*0.25
        elif self.state==States.FWD_RIGHT:
            twist.linear.x = self.forward_speed
            twist.angular.z = -self.turn_speed*0.6
        elif self.state in [States.LEFT, States.ALIGN_LEFT,States.LEFT_LOCK]:
            twist.linear.x = 0
            twist.angular.z = self.turn_speed
        elif self.state in [States.RIGHT, States.ALIGN_RIGHT]:
            twist.linear.x = 0
            twist.angular.z = -self.turn_speed
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        print(f"{self.state} | Line: {(line_fwd,line_L,line_M,line_R)} | Misc: {(red_ln,ped,truck,road_L)}")
        self.drive_pub.publish(twist)

def main():
    rospy.init_node('driver')
    d = Driver()
    rospy.sleep(0.2) 
    d.time_pub.publish(start_timer)
    rospy.spin()
    

if __name__ == '__main__':
    main()
