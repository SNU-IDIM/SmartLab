
from PyQt5.QtWidgets import *
from PyQt5 import uic

import sys, time, rospy
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from syscon_msgs.srv import SaveMap, SaveMapResponse
from syscon_msgs.msg import WorkFlowAction, WorkFlowGoal,Action, Param, RobotState


# Action Modes
WAYPOINT = 0x01
URMISSION = 0x05



class spcore_dialog(QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        uic.loadUi('spcore_simple.ui', self)

        self.ns = self.line_ns.text()

        self.routine_pub = rospy.Publisher("%s/sp_routine"%self.ns,String, queue_size=1)
        self.cmd_pub = rospy.Publisher("%s/cmd_vel"%self.ns,Twist,queue_size=1)

        # Define Client for wait_for_server
        self._mission_client = actionlib.SimpleActionClient('/%s/WAS'%self.ns, WorkFlowAction)
        self._mission_client.wait_for_server(timeout=rospy.Duration(1))
        rospy.loginfo("Connected to WAS")

        # Handler Initialize <NS>
        # self.line_ns.textChanged.connect(self.ns_changed)
        self.btn_ns_edit.clicked.connect(self.ns_changed)

        # Handler Initialize <MODE>
        self.btn_mode_eco.clicked.connect(self.eco_mode_cb)
        self.btn_mode_drive.clicked.connect(self.drive_mode_cb)
        self.btn_mode_draw.clicked.connect(self.draw_mode_cb)
        self.btn_mode_reset.clicked.connect(self.reset_mode_cb)

        # Handler Initialize <CONTROL>
        self.btn_cntl_up.pressed.connect(self.cntl_up_cb)
        self.btn_cntl_dwn.pressed.connect(self.cntl_down_cb)
        self.btn_cntl_left.pressed.connect(self.cntl_left_cb)
        self.btn_cntl_right.pressed.connect(self.cntl_right_cb)
        self.btn_cntl_stop.pressed.connect(self.cntl_stop_cb)

        # Handler Initialize <MAP>
        self.btn_map_save.clicked.connect(self.map_save_cb)

        # Handler Initialize <LOG>
        self.btn_log_clear.clicked.connect(self.log_clear_cb)

        # Handler Initialize <GOAL>
        self.btn_goal_set.clicked.connect(self.goal_set_cb)
        self.btn_save_pose.clicked.connect(self.save_pose_cb)

        # Handler Initialize <ARM>
        self.btn_arm_swing.clicked.connect(self.arm_swing_cb)


    # Handler Definition <NS>
    def ns_changed(self):
        if self.ns == self.line_ns.text(): return
        self.ns = self.line_ns.text()
        self.routine_pub.unregister(); self.cmd_pub.unregister(); del self._mission_client

        self.routine_pub = rospy.Publisher("%s/sp_routine"%self.ns,String, queue_size=1)
        self.cmd_pub = rospy.Publisher("%s/cmd_vel"%self.ns,Twist,queue_size=1)

        self._mission_client = actionlib.SimpleActionClient('/%s/WAS'%self.ns, WorkFlowAction)
        self._mission_client.wait_for_server(timeout=rospy.Duration(1))
        self.log_browser.append("[%s]Namespace Changed to '%s'"%(time.time(),self.line_ns.text()))

    # Handler Definition <MODE>
    def eco_mode_cb(self):
        self.log_browser.append("[%s]changing to 'ECO-mode'..."%time.time())
        self.routine_pub.publish("ECO")
    def drive_mode_cb(self):
        self.log_browser.append("[%s]changing to 'DRIVE-mode'..."%time.time())
        self.routine_pub.publish("NAV")
    def draw_mode_cb(self):
        self.log_browser.append("[%s]changing to 'DRAW-mode'..."%time.time())
        self.routine_pub.publish("SLAM")
    def reset_mode_cb(self):
        self.log_browser.append("[%s]Restarting the robot..."%time.time())
        self.routine_pub.publish("RESET")

    # Handler Definition <MODE>
    def cntl_up_cb(self):
        cntl_lin_vel = self.box_lin_vel.value()
        self.log_browser.append("[%s]publishing speed(%.2f,0.00)"%(time.time(),cntl_lin_vel))
        vel = Twist(); vel.linear.x = cntl_lin_vel
        #self.cmd_pub.publish(vel)
        # print self.btn_cntl_up.pressed
        #rospy.sleep(0.03)
    def cntl_down_cb(self):
        cntl_lin_vel = self.box_lin_vel.value()
        self.log_browser.append("[%s]publishing speed(%.2f,0.00)"%(time.time(),-cntl_lin_vel))
        vel = Twist(); vel.linear.x = -cntl_lin_vel
        #self.cmd_pub.publish(vel)
        #rospy.sleep(0.03)
    def cntl_left_cb(self):
        cntl_ang_vel = self.box_ang_vel.value()
        self.log_browser.append("[%s]publishing speed(0.00,%.2f)"%(time.time(),cntl_ang_vel))
        vel = Twist(); vel.angular.z = cntl_ang_vel
        #self.cmd_pub.publish(vel)
        #rospy.sleep(0.03)
    def cntl_right_cb(self):
        cntl_ang_vel = self.box_ang_vel.value()
        self.log_browser.append("[%s]publishing speed(0.00,%.2f)"%(time.time(),-cntl_ang_vel))
        vel = Twist(); vel.angular.z = -cntl_ang_vel
        #self.cmd_pub.publish(vel)
        #rospy.sleep(0.03)
    def cntl_stop_cb(self):
        self.log_browser.append("[%s]publishing speed(0.00,0.00)"%(time.time()))
        #self.cmd_pub.publish()
        #rospy.sleep(0.03)


    # Service Caller for SaveMap
    def savemap_req(self,rid):
        srv_name = "/%s/save_map"%rid
        try:
            rospy.wait_for_service(srv_name, timeout=0.5)
            srvs = rospy.ServiceProxy(srv_name, SaveMap)
            resp = srvs.call(False)
            return resp
        except rospy.ServiceException, e:
            message = "[%s]Service call failed(call_savemap): %s"%(time.time(),e)
            return SaveMapResponse(False,message,'','')
        except rospy.ROSException, e:
            message = "[%s]TimeOut(call_savemap):%s"%(time.time(),e)
            return SaveMapResponse(False,message,'','')


    # Handler Definition <MAP>
    def map_save_cb(self):
        self.log_browser.append("[%s]Saving a new Map"%time.time())
        resp = self.savemap_req(self.ns)
        if not resp.success: self.log_browser.append(resp.message)
        else: self.log_browser.append(resp.message)

    # Handler Definition <LOG>
    def log_clear_cb(self):
        self.log_browser.clear()

    # Handler Definition <GOAL>
    def goal_set_cb(self):
        self.log_browser.append("[%s]Goal set to %.2f, %.2f, %.2f"
            %(time.time(),self.box_goal_x.value(), self.box_goal_y.value(), self.box_goal_th.value()))

        goal = WorkFlowGoal()

        p = [Param('max_trans_vel','float','0.7'), Param('xy_goal_tolerance','float','0.15'),Param('yaw_goal_tolerance','float','0.05')]
        wp = Action(WAYPOINT, [self.box_goal_x.value(),self.box_goal_y.value(),self.box_goal_th.value()],p)
        goal.work_id = "SET_GOAL"; goal.loop_flag = 1
        goal.work = [wp]
        self._mission_client.send_goal(goal)

    def save_pose_cb(self):
        try:
            r_state = rospy.wait_for_message("/%s/robot_state"%self.ns,RobotState, timeout=0.5)

            self.box_goal_x.setValue(r_state.pose.x)
            self.box_goal_y.setValue(r_state.pose.y)
            self.box_goal_th.setValue(r_state.pose.theta)
            self.log_browser.append("[%s]Save this point - '%.2f, %.2f, %.2f'"
                %(time.time(),self.box_goal_x.value(), self.box_goal_y.value(), self.box_goal_th.value()))
        except Exception, e:
            rospy.logwarn("%s"%e)
            self.log_browser.append("[%s]Can't get the Pose"%(time.time()))

    # Handler Definition <ARM>
    def arm_swing_cb(self):
        URMOVEIT = 0x02; URPNP = 0x03
        PICK, PLACE = 1.0,-1.0
        self.log_browser.append("[%s]Arm Swing Published"%(time.time()))
        goal = WorkFlowGoal()
        arm_cmd = Action(URMISSION, [URPNP, PICK],[])
        goal.work_id = "ARM_SWING"; goal.loop_flag = 1
        goal.work = [arm_cmd]
        self._mission_client.send_goal(goal)


if __name__ == "__main__":
    rospy.init_node("SPCoRE_Simple", anonymous=True)

    app = QApplication(sys.argv)
    spcore_dialog = spcore_dialog()
    spcore_dialog.show()

    app.exec_()
