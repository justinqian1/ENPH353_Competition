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
    STOP_PED_SEEN=auto()
    STOP_TRUCK=auto()
    STOP_END=auto()

start_timer = String('team,pass,0,whatever')
stop_timer = String('team,pass,-1,whatever')

IMG_SZ=320
S1_DATA_LEN=10 # arr of length 10 for sec 1
S2_DATA_LEN=10 # arr of len 8 for sec 2

SPEED_FWD=[1.8,1.6]
SPEED_TURN=[5.0,4.0]
SPEED_XWALK_LOCK=6.0
SPEED_FWD_LEFT_LOCK=4.0
SPEED_ALIGN=1.5

S1_LINE_FWD_BOX_TH=5 # num px in driving box
S1_LINE_EDGE_TOL=30 # diff bw lineL and lineR to go back to moving fwd
LINE_ALIGN_TOL=10 # diff bw lineL and lineR to require aligning
LINE_MID_DIFF_TH=10 # threshold for line middle vs side (to decide to go fwd+left/fwd+right)
LINE_M_TH=190 # y threshold for line middle to matter
S1_LINE_LR_DIFF_TH=30 # diff bw left and right lines to trigger pid driving
RED_LN_TH=2000 # num px of red line needed
S1_PINK_LN_TH=1000 # same for pink line
PED_TH=20 # num px of ped to count as seen
ROAD_SZ_TH=42_000 # num px of road to start seq to enter loop
TRUCK_STOP_TH=1100 # num px in truck to stop for it
TRUCK_RESUME_TH=600 # num px in truck to resume; diff to avoid stop/restart loop
ROAD_L_TH=185 # road y coord to exit line
XWALK_LOCK_TIME=0.35 # time to lock fwd state in crosswalk
ENTER_LOOP_LOCK_TIME=0.35 # lock left turn at start of loop
EXIT_LOOP_LOCK_TIME=0.6 # lock time for fwd left while exiting
EXIT_LOOP_WAIT_TIME=4.0 # wait time before logic to exit loop hits

S2_LINE_FWD_TURN_TH=700 # num px in driving box to go fwd->turn
S2_LINE_FWD_FWD_TH=500 # num px in driving box to go temp stop -> fwd
S2_LINE_EDGE_TOL=18 # diff bw lineL and lineR to go back to moving fwd 
S2_LINE_LR_DIFF_TH=30 # diff bw left and right lines to trigger pid driving
ENTER_BRIDGE_TH=4000 # num water px to enter bridge section
EXIT_BRIDGE_TH=300 # num water px to leave bridge section
WATER_TURN_TH=85 # num pix on either side to start turning
S2_PINK_LN_TH1=50 # threshold to just drive twd pink line
S2_PINK_LN_TH2=150 # threshold to drive at the pink line, regardless of pink ln median
S2_PINK_LN_TH3=2000 # threshold to align self to line
PINK_LN_TOL=20 # tolerance (px in y) to drive straight at the line


class Driver:
    def __init__(self):
        self.data = rospy.Subscriber('/drive_info',Int32MultiArray, callback=self.callback,queue_size=1)
        self.drive_pub = rospy.Publisher('/B1/cmd_vel', Twist, queue_size=1)
        self.loc_pub = rospy.Publisher('/B1/loc', String, queue_size=1)
        self.time_pub = rospy.Publisher('/score_tracker', String, queue_size=1)
        self.section='1' # sections 1,2,3,4
        self.latest_data=None
        self.state = States.FWD 

        self.past_ped=False
        self.exit_loop_time=rospy.Time.now()+rospy.Duration(10000)
        self.past_loop=False
        self.on_bridge=False
        self.past_bridge=False

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
        if self.section=='1' and len(data)==S1_DATA_LEN:
            self.driving_section1(data)
        elif self.section=='2' and len(data)==S2_DATA_LEN:
            self.driving_section2(data)
        else:
            print(f"WARNING: Section: {self.section} but data length: {len(data)}")

        '''
        elapsed = rospy.Time.now() - self.start_time
        if elapsed > self.duration:
            self.state=States.STOP_MAXTIME
            self.drive_pub.publish(Twist())
            self.time_pub.publish(stop_timer)
            return
        '''

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
            elif pink_ln>S1_PINK_LN_TH: # STOP ON PINK LINE -> remove
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
            elif line_fwd>S1_LINE_FWD_BOX_TH: # TURN B/C WE'RE DRIVING AT THE LINE
                self.state=States.STOP_TEMP
            else: # logic to change states
                if self.state == States.FWD: # FWD -> FWD+TURN if either line middle is close, or line is misaligned L/R
                    if (line_M>LINE_M_TH and line_M < line_L-LINE_MID_DIFF_TH) or \
                        (line_L > line_R + S1_LINE_LR_DIFF_TH and line_R > -1):
                        self.state=States.FWD_RIGHT
                    elif (line_M > LINE_M_TH and line_M < line_R-LINE_MID_DIFF_TH) or \
                        (line_R > line_L + S1_LINE_LR_DIFF_TH and line_L > -1):
                        self.state=States.FWD_LEFT
                if (self.state == States.FWD_LEFT and line_L > line_R-S1_LINE_EDGE_TOL) or \
                (self.state == States.FWD_RIGHT and line_R > line_L-S1_LINE_EDGE_TOL):
                    self.state=States.FWD
                
        elif (self.state == States.LEFT and line_L > line_R-S1_LINE_EDGE_TOL) or \
             (self.state == States.RIGHT and line_R > line_L-S1_LINE_EDGE_TOL):
            self.state = States.STOP_TEMP
        
        #align to red/pink line
        elif (self.state == States.ALIGN_LEFT and line_L > line_R-LINE_ALIGN_TOL) or \
             (self.state == States.ALIGN_RIGHT and line_R > line_L-LINE_ALIGN_TOL):
            if pink_ln>S1_PINK_LN_TH: # CASE: pink ln
                self.switch_section('2')
                return
            else: # CASE: PED
                if ped > PED_TH:
                    self.state=States.STOP_PED_SEEN
                else:
                    self.state=States.FWD_LOCK
                    self.past_ped=True
                    self.locked_until = rospy.Time.now() + rospy.Duration(XWALK_LOCK_TIME)
        elif self.state==States.STOP_TEMP:
            if pink_ln>S1_PINK_LN_TH:
                if line_L > line_R+LINE_ALIGN_TOL:
                    self.state=States.ALIGN_RIGHT
                elif line_R > line_L+LINE_ALIGN_TOL:
                    self.state=States.ALIGN_LEFT
                else:
                    self.switch_section('2')
                    return
            elif line_fwd>S1_LINE_FWD_BOX_TH: # fwd -> turn
                self.state=States.RIGHT if line_L > line_R else States.LEFT
            else: # turn -> fwd
                self.state=States.FWD
        elif self.state==States.STOP_PED_SEEN and ped < PED_TH: # ped crossing -> ped done crossing
            self.state=States.FWD_LOCK
            self.past_ped=True
            self.locked_until = rospy.Time.now() + rospy.Duration(XWALK_LOCK_TIME)
        elif self.state==States.STOP_TRUCK and truck < TRUCK_RESUME_TH: # stopped for truck -> restart
            self.state=States.STOP_TEMP
        elif self.state in [States.FWD_LOCK,States.FWD_LEFT_LOCK1, States.FWD_LEFT_LOCK2] and now > self.locked_until: # done locking
            self.state=States.STOP_TEMP

        twist = Twist()
        if self.state == States.FWD:
            twist.linear.x = SPEED_FWD[0]
            twist.angular.z = 0
        elif self.state == States.FWD_LOCK:
            twist.linear.x = SPEED_XWALK_LOCK
            twist.angular.z = 0
        elif self.state==States.FWD_LEFT:
            if line_L==-1: # most aggressive - line middle is close
                ang_speed=SPEED_TURN[0]*0.8
            else: # less aggressive - left and right just not aligned
                ang_speed=SPEED_TURN[0]*0.25
            twist.linear.x = SPEED_FWD[0]
            twist.angular.z = ang_speed
        elif self.state==States.FWD_LEFT_LOCK1: # entering loop
            twist.linear.x = SPEED_FWD_LEFT_LOCK
            twist.angular.z = SPEED_TURN[0]*1.3
        elif self.state==States.FWD_LEFT_LOCK2: # exiting loop
            twist.linear.x = SPEED_FWD_LEFT_LOCK
            twist.angular.z = SPEED_TURN[0]*0.9
        elif self.state==States.FWD_RIGHT:
            if line_L==-1: # most aggressive - line middle is close
                ang_speed=-SPEED_TURN[0]*0.8
            else: # less aggressive - left and right just not aligned
                ang_speed=-SPEED_TURN[0]*0.25
            twist.linear.x = SPEED_FWD[0]
            twist.angular.z = ang_speed
        elif self.state == States.LEFT:
            twist.linear.x = 0
            twist.angular.z = SPEED_TURN[0]
        elif self.state == States.RIGHT:
            twist.linear.x = 0
            twist.angular.z = -SPEED_TURN[0]
        elif self.state == States.ALIGN_LEFT:
            twist.linear.x = 0
            twist.angular.z = SPEED_ALIGN
        elif self.state == States.ALIGN_RIGHT:
            twist.linear.x = 0
            twist.angular.z = -SPEED_ALIGN
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        print(f"{self.state} | Line: {(line_fwd,line_L,line_M,line_R)} | Misc: {(red_ln,pink_ln,ped,truck,road_sz,road_L)}")
        self.drive_pub.publish(twist)
    
    def driving_section2(self,data):
        if not self.on_bridge:
            self.driving_section2_land(data)
        else:
            self.driving_section2_water(data)

    def driving_section2_land(self,data):
        line_fwd,line_L,line_R,amt_L,amt_R,amt_water,land_L,land_R,pink_ln_sz,pink_ln_mid=data
        if amt_water>ENTER_BRIDGE_TH:
            self.on_bridge=True
            self.state=States.FWD
            return
        
        # main driving loop
        if self.state in [States.FWD,States.FWD_LEFT,States.FWD_RIGHT]:
            if line_fwd>S2_LINE_FWD_TURN_TH:
                self.state=States.STOP_TEMP
            else: # logic to change states
                if self.state == States.FWD: # FWD -> FWD+TURN if either line middle is close, or line is misaligned L/R
                    if line_L > line_R + S2_LINE_LR_DIFF_TH and line_R > -1:
                        self.state=States.FWD_RIGHT
                    elif line_R > line_L + S2_LINE_LR_DIFF_TH and line_L > -1:
                        self.state=States.FWD_LEFT
                if (self.state == States.FWD_LEFT and line_L > line_R-S2_LINE_EDGE_TOL) or \
                (self.state == States.FWD_RIGHT and line_R > line_L-S2_LINE_EDGE_TOL):
                    self.state=States.FWD
        elif (self.state == States.LEFT and line_L > line_R-S2_LINE_EDGE_TOL and line_L > -1) or \
            (self.state == States.RIGHT and line_R > line_L-S2_LINE_EDGE_TOL and line_R > -1):
            self.state = States.STOP_TEMP
        elif self.state==States.STOP_TEMP:
            if line_fwd>S2_LINE_FWD_FWD_TH:
                self.state=States.RIGHT if amt_L > amt_R else States.LEFT
            else: # turn -> fwd
                self.state=States.FWD

        # override to drive at pink ln
        if self.past_bridge and pink_ln_sz>S2_PINK_LN_TH1:
            if pink_ln_sz>S2_PINK_LN_TH3:
                self.switch_section('3')
            elif abs(IMG_SZ//2-pink_ln_mid)<PINK_LN_TOL:
                self.state=States.FWD
            elif pink_ln_sz>S2_PINK_LN_TH2:
                self.state=States.FWD_LEFT if pink_ln_mid<IMG_SZ//2 else States.FWD_RIGHT

        twist = Twist()
        if self.state == States.FWD:
            twist.linear.x = SPEED_FWD[1]
            twist.angular.z = 0
        elif self.state == States.LEFT:
            twist.linear.x = 0
            twist.angular.z = SPEED_TURN[1]
        elif self.state == States.RIGHT:
            twist.linear.x = 0
            twist.angular.z = -SPEED_TURN[1]
        elif self.state==States.FWD_LEFT:
            twist.linear.x = SPEED_FWD[1]
            twist.angular.z = SPEED_TURN[1]*0.3 if not self.past_bridge else SPEED_TURN[1]*0.5 # pink line reasons
        elif self.state==States.FWD_RIGHT:
            twist.linear.x = SPEED_FWD[1]
            twist.angular.z = -SPEED_TURN[1]*0.3 if not self.past_bridge else -SPEED_TURN[1]*0.5
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        print(f"{self.state} | Line: {(line_fwd,line_L,line_R,amt_L,amt_R)} | Water: {(amt_water)} | Pink line: {(pink_ln_sz,pink_ln_mid)}")
        self.drive_pub.publish(twist)

    def driving_section2_water(self,data):
        line_fwd,line_L,line_R,amt_L,amt_R,amt_water,land_L,land_R,_,_=data
        if self.on_bridge: # ON BRIDGE: PAY ATTENTION TO WATER
            if amt_water<EXIT_BRIDGE_TH:
                self.on_bridge=False
                self.past_bridge=True
                return
            if self.state == States.FWD:
                if land_L > WATER_TURN_TH:
                    self.state=States.RIGHT
                elif land_R > WATER_TURN_TH:
                    self.state=States.LEFT
            elif (self.state == States.LEFT and land_L > land_R-S2_LINE_EDGE_TOL) or \
                (self.state == States.RIGHT and land_R > land_L-S2_LINE_EDGE_TOL):
                self.state=States.FWD

        twist = Twist()
        if self.state == States.FWD:
            twist.linear.x = SPEED_FWD[1]*0.75
            twist.angular.z = 0
        elif self.state == States.LEFT:
            twist.linear.x = 0
            twist.angular.z = SPEED_TURN[1]*0.75
        elif self.state == States.RIGHT:
            twist.linear.x = 0
            twist.angular.z = -SPEED_TURN[1]*0.75
        else: # stopped
            twist.linear.x = 0
            twist.angular.z = 0
        print(f"{self.state} | Water: {(amt_water,land_L,land_R)}")
        self.drive_pub.publish(twist)
    
    def switch_section(self,new_section):
        self.state=States.STOP_TEMP
        self.drive_pub.publish(Twist())
        self.section=new_section
        self.loc_pub.publish(new_section)
        #'''
        if new_section=='3':
            self.state=States.STOP_END
            self.time_pub.publish(stop_timer)
        #'''

def main():
    rospy.init_node('driver')
    d = Driver()
    rospy.sleep(0.2) 
    d.time_pub.publish(start_timer)
    rospy.spin()
    

if __name__ == '__main__':
    main()
