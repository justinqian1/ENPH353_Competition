#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String
from enum import Enum, auto

class States(Enum):
    FWD=auto()
    FWD_LOCK=auto()
    LEFT=auto()
    RIGHT=auto()
    FWD_LEFT=auto()
    FWD_LEFT_LOCK1=auto()
    FWD_LEFT_LOCK2=auto()
    FWD_RIGHT=auto()
    ALIGN_LEFT=auto()
    ALIGN_RIGHT=auto()
    STOP_TEMP=auto() # only for transitioning bw fwd/turn
    STOP_PED_NOTSEEN=auto()
    STOP_PED_SEEN=auto()
    STOP_TRUCK=auto()
    STOP_END=auto()

start_timer = String('team,pass,0,whatever')
stop_timer = String('team,pass,-1,whatever')

LINE_FWD_BOX_TH=5 # num px in driving box
LINE_EDGE_TOL=30 # diff bw lineL and lineR to go back to moving fwd
LINE_ALIGN_TOL=10 # diff bw lineL and lineR to require aligning
LINE_MID_DIFF_TH=10 # threshold for line middle vs side (to decide to go fwd+left/fwd+right)
LINE_M_TH=190 # y threshold for line middle to matter
LINE_M_LOOP_TH=200 # line middle in the loop (i.e. begin hardcode truck portion)
LINE_LR_LOOP_TH=200 # left and right thresholds for loop
LINE_LR_DIFF_TH=30 # diff bw left and right lines to trigger pid driving
RED_LN_TH=2000 # num px of red line needed
PINK_LN_TH=1000 # same for pink line
PED_TH=20 # num px of ped to count as seen
ROAD_SZ_TH=42_000 # num px of road to start seq to enter loop
TRUCK_STOP_TH=1100 # num px in truck to stop for it
TRUCK_RESUME_TH=600 # num px in truck to resume; diff to avoid stop/restart loop
ROAD_L_TH=185 # road y coord to exit line
XWALK_LOCK_TIME=0.35 # time to lock fwd state in crosswalk
ENTER_LOOP_LOCK_TIME=0.35 # lock left turn at start of loop
EXIT_LOOP_LOCK_TIME=0.6 # lock time for fwd left while exiting
EXIT_LOOP_WAIT_TIME=4.0 # wait time before logic to exit loop hits

class Driver:
    def __init__(self):
        self.data = rospy.Subscriber('/drive_info',Int32MultiArray, callback=self.callback,queue_size=1)
        self.drive_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.loc_pub = rospy.Publisher('/B1/loc', String, queue_size=1)
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.section=1 # sections 1,2,3,
        self.latest_data=None
        self.state = States.FWD 

        self.past_ped=False
        self.exit_loop_time=rospy.Time.now()+rospy.Duration(10000)
        self.past_loop=False

        self.forward_speed = 2.0
        self.fwd_lock_speed = 6.0 # it's locked anyway
        self.fwd_left_lock_lin_speed = 4.0
        self.turn_speed = 6.0
        self.align_speed = 1.5

        self.timer = rospy.Timer(rospy.Duration(0.05), self.drive)

        self.time_pub.publish(start_timer)
        #self.start_time = rospy.Time.now()
        #self.duration = rospy.Duration(120.0) # drive for 10 s

    def callback(self, msg):
        self.latest_data=msg.data
    
    def drive(self,event):
        data=self.latest_data
        if data is None:
            return
        if self.section==1:
            self.driving_section1(data)
        elif self.section==2:
            self.driving_section2(data)

    def driving_section1(self,data):
        line_fwd,line_L,line_M,line_R,red_ln,pink_ln,ped,truck,road_sz,road_L=data
        now=rospy.Time.now()
        if self.state in [States.FWD,States.FWD_LEFT,States.FWD_RIGHT]:
            if red_ln>RED_LN_TH and not self.past_ped: # STOP FOR PED
                if line_L > line_R+LINE_ALIGN_TOL:
                    self.state=States.ALIGN_RIGHT
                elif line_R > line_L+LINE_ALIGN_TOL:
                    self.state=States.ALIGN_LEFT
                elif ped > PED_TH:
                    self.state=States.STOP_PED_SEEN
                else: # dont need to align left or right, or stop for 
                    self.state=States.FWD_LOCK
                    self.past_ped=True
                    self.locked_until = rospy.Time.now() + rospy.Duration(XWALK_LOCK_TIME)
            elif pink_ln>PINK_LN_TH: # STOP ON PINK LINE -> remove
                self.state=States.STOP_TEMP
            elif truck>TRUCK_STOP_TH and self.past_ped and not self.past_loop: # STOP FOR TRUCK
                self.state=States.STOP_TRUCK
            elif line_L==-1 and line_M==-1 and line_R==-1 and road_sz>ROAD_SZ_TH: # ENTERING LOOP
                self.state=States.FWD_LEFT_LOCK1
                self.locked_until = now + rospy.Duration(ENTER_LOOP_LOCK_TIME)
                self.exit_loop_time = now+rospy.Duration(EXIT_LOOP_WAIT_TIME)
            # line_M>LINE_M_LOOP_TH and line_L>LINE_LR_LOOP_TH and line_R>LINE_LR_LOOP_TH: # OLD CONDITION FOR ENTER
            elif road_L< ROAD_L_TH and now > self.exit_loop_time and not self.past_loop: # EXITING LOOP
                self.state=States.FWD_LEFT_LOCK2
                self.past_loop=True
                self.locked_until = now + rospy.Duration(EXIT_LOOP_LOCK_TIME)
            elif line_fwd>LINE_FWD_BOX_TH: # TURN B/C WE'RE DRIVING AT THE LINE
                self.state=States.STOP_TEMP
            else: # logic to change states
                if self.state == States.FWD: # FWD -> FWD+TURN if either line middle is close, or line is misaligned L/R
                    if (line_M>LINE_M_TH and line_M < line_L-LINE_MID_DIFF_TH) or \
                        (line_L > line_R + LINE_LR_DIFF_TH and line_R > -1):
                        self.state=States.FWD_RIGHT
                    elif (line_M > LINE_M_TH and line_M < line_R-LINE_MID_DIFF_TH) or \
                        (line_R > line_L + LINE_LR_DIFF_TH and line_L > -1):
                        self.state=States.FWD_LEFT
                if (self.state == States.FWD_LEFT and line_L > line_R-LINE_EDGE_TOL) or \
                (self.state == States.FWD_RIGHT and line_R > line_L-LINE_EDGE_TOL):
                    self.state=States.FWD
                
        elif (self.state == States.LEFT and line_L > line_R-LINE_EDGE_TOL) or \
             (self.state == States.RIGHT and line_R > line_L-LINE_EDGE_TOL):
            self.state = States.STOP_TEMP
        
        #align to red/pink line
        elif (self.state == States.ALIGN_LEFT and line_L > line_R-LINE_ALIGN_TOL) or \
             (self.state == States.ALIGN_RIGHT and line_R > line_L-LINE_ALIGN_TOL):
            if pink_ln>PINK_LN_TH: # CASE: pink ln
                self.state=States.STOP_END
                self.time_pub.publish(stop_timer)
                self.loc_pub.publish('1')
            else: # CASE: PED
                if ped > PED_TH:
                    self.state=States.STOP_PED_SEEN
                else:
                    self.state=States.FWD_LOCK
                    self.past_ped=True
                    self.locked_until = rospy.Time.now() + rospy.Duration(XWALK_LOCK_TIME)
        elif self.state==States.STOP_TEMP:
            if pink_ln>PINK_LN_TH:
                if line_L > line_R+LINE_ALIGN_TOL:
                    self.state=States.ALIGN_RIGHT
                elif line_R > line_L+LINE_ALIGN_TOL:
                    self.state=States.ALIGN_LEFT
                else:
                    self.state=States.STOP_END
                    self.time_pub.publish(stop_timer)
            elif line_fwd>LINE_FWD_BOX_TH: # fwd -> turn
                self.state=States.RIGHT if line_L > line_R else States.LEFT
            else: # turn -> fwd
                self.state=States.FWD
        elif self.state==States.STOP_PED_NOTSEEN and ped > PED_TH: # ped not yet crossed -> ped crossing
            self.state=States.STOP_PED_SEEN
        elif self.state==States.STOP_PED_SEEN and ped < PED_TH: # ped crossing -> ped done crossing
            self.state=States.FWD_LOCK
            self.past_ped=True
            self.locked_until = rospy.Time.now() + rospy.Duration(XWALK_LOCK_TIME)
        elif self.state==States.STOP_TRUCK and truck < TRUCK_RESUME_TH: # stopped for truck -> restart
            self.state=States.STOP_TEMP
        elif self.state in [States.FWD_LOCK,States.FWD_LEFT_LOCK1, States.FWD_LEFT_LOCK2] and now > self.locked_until: # done locking
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
        if self.state == States.FWD:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0
        elif self.state == States.FWD_LOCK:
            twist.linear.x = self.fwd_lock_speed
            twist.angular.z = 0
        elif self.state==States.FWD_LEFT:
            if line_L==-1: # most aggressive - line middle is close
                ang_speed=self.turn_speed*0.8
            else: # less aggressive - left and right just not aligned
                ang_speed=self.turn_speed*0.25
            twist.linear.x = self.forward_speed
            twist.angular.z = ang_speed
        elif self.state==States.FWD_LEFT_LOCK1:
            twist.linear.x = self.fwd_left_lock_lin_speed
            twist.angular.z = self.turn_speed*0.9
        elif self.state==States.FWD_LEFT_LOCK2:
            twist.linear.x = self.fwd_left_lock_lin_speed
            twist.angular.z = self.turn_speed*0.6
        elif self.state==States.FWD_RIGHT:
            if line_L==-1: # most aggressive - line middle is close
                ang_speed=-self.turn_speed*0.8
            else: # less aggressive - left and right just not aligned
                ang_speed=-self.turn_speed*0.25
            twist.linear.x = self.forward_speed
            twist.angular.z = ang_speed
        elif self.state == States.LEFT:
            twist.linear.x = 0
            twist.angular.z = self.turn_speed
        elif self.state == States.RIGHT:
            twist.linear.x = 0
            twist.angular.z = -self.turn_speed
        elif self.state == States.ALIGN_LEFT:
            twist.linear.x = 0
            twist.angular.z = self.align_speed
        elif self.state == States.ALIGN_RIGHT:
            twist.linear.x = 0
            twist.angular.z = -self.align_speed
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        print(f"{self.state} | Line: {(line_fwd,line_L,line_M,line_R)} | Misc: {(red_ln,pink_ln,ped,truck,road_sz,road_L)}")
        self.drive_pub.publish(twist)

def main():
    rospy.init_node('driver')
    d = Driver()
    rospy.sleep(0.2) 
    d.time_pub.publish(start_timer)
    rospy.spin()
    

if __name__ == '__main__':
    main()