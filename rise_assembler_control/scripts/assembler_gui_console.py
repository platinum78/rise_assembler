#!/usr/bin/python2
from PyQt5.QtWidgets import QDialog, QApplication, QMainWindow
from PyQt5.QtCore import pyqtSlot
from ui.kinematics_setup import *
from ui.current_task import *
from ui.mainwindow import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os, sys, copy
from assembler_controller import *

class ControlPanel(QMainWindow):
    def __init__(self):
        super(ControlPanel, self).__init__()
        self.ros_moveit_setup()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.widgets_connection_setup()
        self.show()
    
    def ros_moveit_setup(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("assembler_controller_node")

        self.assembler_cmd = moveit_commander.RobotCommander()
        self.abb_irb120 = moveit_commander.MoveGroupCommander("abb_irb120")
        self.assembler = rise_assembler_controller(self.assembler_cmd, self.abb_irb120)
        # assembler.move_to_test_pos()
        rospy.loginfo("SYSTEM READY!!!")
    
    @pyqtSlot()
    def arm1_step_by_linear_path(self, mode, direction, sign):
        self.ui.arm1_joystick.setDisabled(True)

        if mode == "fine":
            if direction == "x":
                step = self.ui.fine_x.value() / 1000
            elif direction == "y":
                step = self.ui.fine_y.value() / 1000
            elif direction == "z":
                step = self.ui.fine_z.value() / 1000
        elif mode == "normal":
            if direction == "x":
                step = self.ui.normal_x.value() / 1000
            elif direction == "y":
                step = self.ui.normal_y.value() / 1000
            elif direction == "z":
                step = self.ui.normal_z.value() / 1000
        elif mode == "large":
            if direction == "x":
                step = self.ui.large_x.value() / 1000
            elif direction == "y":
                step = self.ui.large_y.value() / 1000
            elif direction == "z":
                step = self.ui.large_z.value() / 1000
        
        print "Move %f to %s direction" % (step, direction)

        pose_now = self.assembler.abb_irb120.get_current_pose().pose
        pose_new = copy.deepcopy(pose_now)
        if sign == "minus":
            step *= -1
        if direction == "x":
            pose_new.position.x += step
        elif direction == "y":
            pose_new.position.y += step
        elif direction == "z":
            pose_new.position.z += step
        
        pose = [0] * 6
        pose[0] = pose_new.position.x
        pose[1] = pose_new.position.y
        pose[2] = pose_new.position.z
        q = pose_new.orientation
        rpy = euler_from_quaternion([q.x, q.y, q.z, q.w])
        pose[3] = rpy[0]
        pose[4] = rpy[1]
        pose[5] = rpy[2]
        self.assembler.move_by_cartesian_path(pose)
        self.ui.arm1_joystick.setDisabled(False)
    
    @pyqtSlot()
    def arm1_move_to_init_pose(self):
        self.ui.arm1_joystick.setDisabled(True)
        self.assembler.move_to_init_pos()
        self.ui.arm1_joystick.setDisabled(False)
    
    @pyqtSlot()
    def arm1_update_pose(self):
        p = self.assembler.get_pose()
        p[3:6] = [x * 180 / np.pi for x in p[3:6]]

        self.ui.arm1_x.setText("%.6f" % p[0])
        self.ui.arm1_y.setText("%.6f" % p[1])
        self.ui.arm1_z.setText("%.6f" % p[2])
        self.ui.arm1_r.setText("%.6f" % p[3])
        self.ui.arm1_p.setText("%.6f" % p[4])
        self.ui.arm1_y_.setText("%.6f" % p[5])
    
    @pyqtSlot()
    def arm1_set_gripper_pos(self, mode):
        if mode == "grip":
            gripper_pos = self.ui.arm1_spinBox_gripper_grip.value()
        elif mode == "open":
            gripper_pos = self.ui.arm1_spinBox_gripper_open.value()
        self.assembler.set_gripper_pos(gripper_pos)

    @pyqtSlot()
    def arm1_move_by_linear_path(self, motion, mode):
        pose = self.assembler.get_pose()

        if motion == "position" or motion == "pose":
            if self.ui.arm1_xyz_meters.isChecked() and not self.ui.arm1_xyz_millimeters.isChecked():
                factor = 1
            elif not self.ui.arm1_xyz_meters.isChecked() and self.ui.arm1_xyz_millimeters.isChecked():
                factor = 0.001

            if mode == "delta":
                pose[0] += self.ui.arm1_nc_x.value() * factor
                pose[1] += self.ui.arm1_nc_y.value() * factor
                pose[2] += self.ui.arm1_nc_z.value() * factor
            elif mode == "goto":
                pose[0] = self.ui.arm1_nc_x.value() * factor
                pose[1] = self.ui.arm1_nc_y.value() * factor
                pose[2] = self.ui.arm1_nc_z.value() * factor

        if motion == "orientation" or motion == "pose":
            if self.ui.arm1_rpy_degrees.isChecked() and not self.ui.arm1_rpy_radians.isChecked():
                factor = np.pi / 180
            elif not self.ui.arm1_rpy_degrees.isChecked() and self.ui.arm1_rpy_radians.isChecked():
                factor = 1

            if mode == "delta":
                pose[3] += self.ui.arm1_nc_r.value() * factor
                pose[4] += self.ui.arm1_nc_p.value() * factor
                pose[5] += self.ui.arm1_nc_y_.value() * factor
            elif mode == "goto":
                pose[3] = self.ui.arm1_nc_r.value() * factor
                pose[4] = self.ui.arm1_nc_p.value() * factor
                pose[5] = self.ui.arm1_nc_y_.value() * factor
        
        self.assembler.move_by_cartesian_path(pose)
    
    @pyqtSlot()
    def arm1_nc_get_pose(self):
        # Check selected units
        if self.ui.arm1_xyz_meters.isChecked() and not self.ui.arm1_xyz_millimeters.isChecked():
            length_factor = 1
            decimals = 6
        elif not self.ui.arm1_xyz_meters.isChecked() and self.ui.arm1_xyz_millimeters.isChecked():
            length_factor = 1000
            decimals = 2
        if self.ui.arm1_rpy_degrees.isChecked() and not self.ui.arm1_rpy_radians.isChecked():
            angle_factor = 180 / np.pi
        elif not self.ui.arm1_rpy_degrees.isChecked() and self.ui.arm1_rpy_radians.isChecked():
            angle_factor = 1
        
        pose = self.assembler.get_pose()
        pose[0] *= length_factor
        pose[1] *= length_factor
        pose[2] *= length_factor
        pose[3] *= angle_factor
        pose[4] *= angle_factor
        pose[5] *= angle_factor
        
        # Set values to each widget
        self.ui.arm1_nc_x.setValue(pose[0])
        self.ui.arm1_nc_y.setValue(pose[1])
        self.ui.arm1_nc_z.setValue(pose[2])
        self.ui.arm1_nc_r.setValue(pose[3])
        self.ui.arm1_nc_p.setValue(pose[4])
        self.ui.arm1_nc_y_.setValue(pose[5])

        # Change decimals of widgets
        self.ui.arm1_nc_x.setDecimals(decimals)
        self.ui.arm1_nc_y.setDecimals(decimals)
        self.ui.arm1_nc_z.setDecimals(decimals)
    
    @pyqtSlot()
    def arm1_nc_reset_fields(self):
        self.ui.arm1_nc_x.setValue(0)
        self.ui.arm1_nc_y.setValue(0)
        self.ui.arm1_nc_z.setValue(0)
        self.ui.arm1_nc_r.setValue(0)
        self.ui.arm1_nc_p.setValue(0)
        self.ui.arm1_nc_y_.setValue(0)

    @pyqtSlot()
    def arm1_joints_slider_to_spinbox(self, from_):
        if from_ == "slider1":
            tx = self.ui.arm1_slider_joint1_angle
            rx = self.ui.arm1_spinbox_joint1_angle
        elif from_ == "slider2":
            tx = self.ui.arm1_slider_joint2_angle
            rx = self.ui.arm1_spinbox_joint2_angle
        elif from_ == "slider3":
            tx = self.ui.arm1_slider_joint3_angle
            rx = self.ui.arm1_spinbox_joint3_angle
        elif from_ == "slider4":
            tx = self.ui.arm1_slider_joint4_angle
            rx = self.ui.arm1_spinbox_joint4_angle
        elif from_ == "slider5":
            tx = self.ui.arm1_slider_joint5_angle
            rx = self.ui.arm1_spinbox_joint5_angle
        elif from_ == "slider6":
            tx = self.ui.arm1_slider_joint6_angle
            rx = self.ui.arm1_spinbox_joint6_angle
        
        val = int(tx.value())
        rx.setValue(val)

    @pyqtSlot()
    def arm1_joints_spinbox_to_slider(self, from_):
        if from_ == "spinbox1":
            tx = self.ui.arm1_spinbox_joint1_angle
            rx = self.ui.arm1_slider_joint1_angle
        elif from_ == "spinbox2":
            tx = self.ui.arm1_spinbox_joint2_angle
            rx = self.ui.arm1_slider_joint2_angle
        elif from_ == "spinbox3":
            tx = self.ui.arm1_spinbox_joint3_angle
            rx = self.ui.arm1_slider_joint3_angle
        elif from_ == "spinbox4":
            tx = self.ui.arm1_spinbox_joint4_angle
            rx = self.ui.arm1_slider_joint4_angle
        elif from_ == "spinbox5":
            tx = self.ui.arm1_spinbox_joint5_angle
            rx = self.ui.arm1_slider_joint5_angle
        elif from_ == "spinbox6":
            tx = self.ui.arm1_spinbox_joint6_angle
            rx = self.ui.arm1_slider_joint6_angle

        val = float(tx.value())
        rx.setValue(val)

    @pyqtSlot()
    def arm1_joints_get_angles(self):
        angles = self.assembler.abb_irb120.get_current_joint_values()
        self.ui.arm1_spinbox_joint1_angle.setValue(angles[0] * 180 / np.pi)
        self.ui.arm1_spinbox_joint2_angle.setValue(angles[1] * 180 / np.pi)
        self.ui.arm1_spinbox_joint3_angle.setValue(angles[2] * 180 / np.pi)
        self.ui.arm1_spinbox_joint4_angle.setValue(angles[3] * 180 / np.pi)
        self.ui.arm1_spinbox_joint5_angle.setValue(angles[4] * 180 / np.pi)
        self.ui.arm1_spinbox_joint6_angle.setValue(angles[5] * 180 / np.pi)
        self.ui.arm1_slider_joint1_angle.setValue(int(angles[0] * 180 / np.pi))
        self.ui.arm1_slider_joint2_angle.setValue(int(angles[1] * 180 / np.pi))
        self.ui.arm1_slider_joint3_angle.setValue(int(angles[2] * 180 / np.pi))
        self.ui.arm1_slider_joint4_angle.setValue(int(angles[3] * 180 / np.pi))
        self.ui.arm1_slider_joint5_angle.setValue(int(angles[4] * 180 / np.pi))
        self.ui.arm1_slider_joint6_angle.setValue(int(angles[5] * 180 / np.pi))
    
    @pyqtSlot()
    def arm1_joints_reset_angles(self):
        self.ui.arm1_spinbox_joint1_angle.setValue(0)
        self.ui.arm1_spinbox_joint2_angle.setValue(0)
        self.ui.arm1_spinbox_joint3_angle.setValue(0)
        self.ui.arm1_spinbox_joint4_angle.setValue(0)
        self.ui.arm1_spinbox_joint5_angle.setValue(0)
        self.ui.arm1_spinbox_joint6_angle.setValue(0)
        self.ui.arm1_slider_joint1_angle.setValue(0)
        self.ui.arm1_slider_joint2_angle.setValue(0)
        self.ui.arm1_slider_joint3_angle.setValue(0)
        self.ui.arm1_slider_joint4_angle.setValue(0)
        self.ui.arm1_slider_joint5_angle.setValue(0)
        self.ui.arm1_slider_joint6_angle.setValue(0)
    
    @pyqtSlot()
    def arm1_joints_move_to_angle(self, mode):
        joint_angles = self.assembler.abb_irb120.get_current_joint_values()
        if mode == "delta":
            joint_angles[0] += self.ui.arm1_spinbox_joint1_angle.value() * np.pi / 180
            joint_angles[1] += self.ui.arm1_spinbox_joint2_angle.value() * np.pi / 180
            joint_angles[2] += self.ui.arm1_spinbox_joint3_angle.value() * np.pi / 180
            joint_angles[3] += self.ui.arm1_spinbox_joint4_angle.value() * np.pi / 180
            joint_angles[4] += self.ui.arm1_spinbox_joint5_angle.value() * np.pi / 180
            joint_angles[5] += self.ui.arm1_spinbox_joint6_angle.value() * np.pi / 180
        elif mode == "goto":
            joint_angles[0] = self.ui.arm1_spinbox_joint1_angle.value() * np.pi / 180
            joint_angles[1] = self.ui.arm1_spinbox_joint2_angle.value() * np.pi / 180
            joint_angles[2] = self.ui.arm1_spinbox_joint3_angle.value() * np.pi / 180
            joint_angles[3] = self.ui.arm1_spinbox_joint4_angle.value() * np.pi / 180
            joint_angles[4] = self.ui.arm1_spinbox_joint5_angle.value() * np.pi / 180
            joint_angles[5] = self.ui.arm1_spinbox_joint6_angle.value() * np.pi / 180
        self.assembler.move_to_joint_pos(joint_angles)
    
    @pyqtSlot()
    def pose_record(self):
        pose_degree = self.assembler.get_pose()
        pose_radian = self.assembler.get_pose()

        pose_degree[3] *= 180 / np.pi
        pose_degree[4] *= 180 / np.pi
        pose_degree[5] *= 180 / np.pi

        self.ui.pose_copy_degree.setText(" ".join([str(x) for x in pose_degree]))
        self.ui.pose_copy_radian.setText(" ".join([str(x) for x in pose_radian]))


    def widgets_connection_setup(self):
        ################################################################################
        # ARM 1 JOYSTICK BUTTONS
        ################################################################################
        self.ui.arm1_button_x_plus_fine.clicked.connect(lambda: self.arm1_step_by_linear_path("fine", "x", "plus"))
        self.ui.arm1_button_y_plus_fine.clicked.connect(lambda: self.arm1_step_by_linear_path("fine", "y", "plus"))
        self.ui.arm1_button_z_plus_fine.clicked.connect(lambda: self.arm1_step_by_linear_path("fine", "z", "plus"))
        self.ui.arm1_button_x_minus_fine.clicked.connect(lambda: self.arm1_step_by_linear_path("fine", "x", "minus"))
        self.ui.arm1_button_y_minus_fine.clicked.connect(lambda: self.arm1_step_by_linear_path("fine", "y", "minus"))
        self.ui.arm1_button_z_minus_fine.clicked.connect(lambda: self.arm1_step_by_linear_path("fine", "z", "minus"))
        self.ui.arm1_button_x_plus_normal.clicked.connect(lambda: self.arm1_step_by_linear_path("normal", "x", "plus"))
        self.ui.arm1_button_y_plus_normal.clicked.connect(lambda: self.arm1_step_by_linear_path("normal", "y", "plus"))
        self.ui.arm1_button_z_plus_normal.clicked.connect(lambda: self.arm1_step_by_linear_path("normal", "z", "plus"))
        self.ui.arm1_button_x_minus_normal.clicked.connect(lambda: self.arm1_step_by_linear_path("normal", "x", "minus"))
        self.ui.arm1_button_y_minus_normal.clicked.connect(lambda: self.arm1_step_by_linear_path("normal", "y", "minus"))
        self.ui.arm1_button_z_minus_normal.clicked.connect(lambda: self.arm1_step_by_linear_path("normal", "z", "minus"))
        self.ui.arm1_button_x_plus_large.clicked.connect(lambda: self.arm1_step_by_linear_path("large", "x", "plus"))
        self.ui.arm1_button_y_plus_large.clicked.connect(lambda: self.arm1_step_by_linear_path("large", "y", "plus"))
        self.ui.arm1_button_z_plus_large.clicked.connect(lambda: self.arm1_step_by_linear_path("large", "z", "plus"))
        self.ui.arm1_button_x_minus_large.clicked.connect(lambda: self.arm1_step_by_linear_path("large", "x", "minus"))
        self.ui.arm1_button_y_minus_large.clicked.connect(lambda: self.arm1_step_by_linear_path("large", "y", "minus"))
        self.ui.arm1_button_z_minus_large.clicked.connect(lambda: self.arm1_step_by_linear_path("large", "z", "minus"))
        self.ui.arm1_button_init_pose.clicked.connect(self.arm1_move_to_init_pose)
        self.ui.arm1_button_gripper_grip.clicked.connect(lambda: self.arm1_set_gripper_pos("grip"))
        self.ui.arm1_button_gripper_open.clicked.connect(lambda: self.arm1_set_gripper_pos("open"))
        ################################################################################

        ################################################################################
        # ARM 1 NUMERICAL CONTROL BUTTONS
        ################################################################################
        self.ui.arm1_button_delta_position.clicked.connect(lambda: self.arm1_move_by_linear_path("position", "delta"))
        self.ui.arm1_button_goto_position.clicked.connect(lambda: self.arm1_move_by_linear_path("position", "goto"))
        self.ui.arm1_button_delta_orientation.clicked.connect(lambda: self.arm1_move_by_linear_path("orientation", "delta"))
        self.ui.arm1_button_goto_orientation.clicked.connect(lambda: self.arm1_move_by_linear_path("orientation", "goto"))
        self.ui.arm1_button_delta_pose.clicked.connect(lambda: self.arm1_move_by_linear_path("pose", "delta"))
        self.ui.arm1_button_goto_pose.clicked.connect(lambda: self.arm1_move_by_linear_path("pose", "goto"))
        self.ui.arm1_get_pose.clicked.connect(lambda: self.arm1_nc_get_pose())
        self.ui.arm1_reset_pose.clicked.connect(lambda: self.arm1_nc_reset_fields())
        
        ################################################################################
        # ARM 1 CONTROL BUTTONS
        ################################################################################
        self.ui.arm1_update_pose.clicked.connect(self.arm1_update_pose)
        self.ui.arm1_button_get_joint_angles.clicked.connect(self.arm1_joints_get_angles)
        self.ui.arm1_button_reset_angles.clicked.connect(self.arm1_joints_reset_angles)
        self.ui.arm1_delta_angle.clicked.connect(lambda: self.arm1_joints_move_to_angle("delta"))
        self.ui.arm1_goto_angle.clicked.connect(lambda: self.arm1_joints_move_to_angle("goto"))
        ################################################################################

        ################################################################################
        # ARM 1 CONTROL EVENTS
        ################################################################################
        self.ui.arm1_slider_joint1_angle.valueChanged.connect(lambda: self.arm1_joints_slider_to_spinbox("slider1"))
        self.ui.arm1_slider_joint2_angle.valueChanged.connect(lambda: self.arm1_joints_slider_to_spinbox("slider2"))
        self.ui.arm1_slider_joint3_angle.valueChanged.connect(lambda: self.arm1_joints_slider_to_spinbox("slider3"))
        self.ui.arm1_slider_joint4_angle.valueChanged.connect(lambda: self.arm1_joints_slider_to_spinbox("slider4"))
        self.ui.arm1_slider_joint5_angle.valueChanged.connect(lambda: self.arm1_joints_slider_to_spinbox("slider5"))
        self.ui.arm1_slider_joint6_angle.valueChanged.connect(lambda: self.arm1_joints_slider_to_spinbox("slider6"))
        self.ui.arm1_spinbox_joint1_angle.valueChanged.connect(lambda: self.arm1_joints_spinbox_to_slider("spinbox1"))
        self.ui.arm1_spinbox_joint2_angle.valueChanged.connect(lambda: self.arm1_joints_spinbox_to_slider("spinbox2"))
        self.ui.arm1_spinbox_joint3_angle.valueChanged.connect(lambda: self.arm1_joints_spinbox_to_slider("spinbox3"))
        self.ui.arm1_spinbox_joint4_angle.valueChanged.connect(lambda: self.arm1_joints_spinbox_to_slider("spinbox4"))
        self.ui.arm1_spinbox_joint5_angle.valueChanged.connect(lambda: self.arm1_joints_spinbox_to_slider("spinbox5"))
        self.ui.arm1_spinbox_joint6_angle.valueChanged.connect(lambda: self.arm1_joints_spinbox_to_slider("spinbox6"))
        ################################################################################

        ################################################################################
        # POSITION RECORD TAB BUTTONS
        ################################################################################
        self.ui.button_record_pose.clicked.connect(self.pose_record)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = ControlPanel()
    w.show()
    sys.exit(app.exec_())