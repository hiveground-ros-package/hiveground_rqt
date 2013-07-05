"""
/*
 * Copyright (c) 2013, HiveGround Co., Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the HiveGround Co., Ltd., nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 *
 */
"""


import os
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import *


import roslib; roslib.load_manifest("rqt_interactive_probe");
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import tf
import copy
import moveit_msgs.srv
import moveit_msgs.msg
import control_msgs.msg
import trajectory_msgs.msg
import actionlib


class MarkerInfo:
    def __init__(self, pose, length, scale):       
        self.pose = pose
        self.length = length
        self.scale = scale
        self.selected = False


class InteractiveProbe(Plugin):
       
    def __init__(self, context):
        super(InteractiveProbe, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('InteractiveProbe')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_interactive_probe'), 'resource', 'InteractiveProbe.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('InteractiveProbeUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        
        self._widget.add_marker_button.clicked.connect(self._addMarker)        
        self._widget.add_at_selected_button.clicked.connect(self._addAtSelectedMarker)
        self._widget.delete_button.clicked.connect(self._deleteSelectedMarker)
        self._widget.clear_button.clicked.connect(self._clearAllMarkers)      
        self._widget.joy_topic_check_box.toggled.connect(self._enableJoyMessageToggled)
        self._widget.reset_rotation.clicked.connect(self._resetRotation)
        self._widget.plan_button.clicked.connect(self._planMotion)
        
        
        context.add_widget(self._widget)
    
        
        self.server = InteractiveMarkerServer("InteractiveProbe")
        self.fixed_frame = "/base_link"              
        self.marker_id = 0
        self.marker_info = dict()
        self.selected_marker = ""
        self.last_button_state = [0 for i in range(16)]
        self.arm_is_active = False
        
        
           
        #rospy.wait_for_service("compute_ik")
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("base_link", "link6", rospy.Time(0), rospy.Duration(5.0))
        (trans,rot) = self.listener.lookupTransform("base_link", "link6", rospy.Time(0))
        rospy.loginfo(trans)
        rospy.loginfo(rot)
        self.start_trans = trans
        self.start_rot = rot
        
        self._widget.x_spin_box.setValue(trans[0])
        self._widget.y_spin_box.setValue(trans[1])
        self._widget.z_spin_box.setValue(trans[2])                        
        euler = tf.transformations.euler_from_quaternion(rot)
        self._widget.rx_spin_box.setValue(euler[0]) 
        self._widget.ry_spin_box.setValue(euler[1])
        self._widget.rz_spin_box.setValue(euler[2])

        #Refresh all related topics
        self.refreshTopics()
        
        self.current_joint_state = sensor_msgs.msg.JointState()
        self.joint_state_subscriber = rospy.Subscriber("/midem/joint_states", sensor_msgs.msg.JointState, self.callbackJointState)
                           
        self.action_client = actionlib.SimpleActionClient("/midem/ptp_controller/ptp_follow_joint_trajectory", control_msgs.msg.FollowJointTrajectoryAction)
        self.action_client.wait_for_server(rospy.Duration(1.0))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        rospy.loginfo("shutdown")
        try:
            self.joy_subscriber
        except:
            pass
        else:
            self.joy_subscriber.unregister()
        
        
        self.server.clear()
        self.server.applyChanges()        

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass        

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass        

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure it
        # Usually used to open a configuration dialog                                                        
                                 
    def _addMarker(self):        
        pose = geometry_msgs.msg.Pose()
        pose.position.x = self._widget.x_spin_box.value()
        pose.position.y = self._widget.y_spin_box.value()
        pose.position.z = self._widget.z_spin_box.value()        
        q = tf.transformations.quaternion_from_euler(self._widget.rx_spin_box.value(), 
                                                     self._widget.ry_spin_box.value(),
                                                     self._widget.rz_spin_box.value())
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]        
        scale = self._widget.scale_spin_box.value()
        length = self._widget.length_spin_box.value()
        name = "marker" + str(self.marker_id)
        self.marker_id += 1
        self.addMaker(name, pose, True, length, scale)
        self.selectOnlyOneMarker(name)
        
    def _addAtSelectedMarker(self):
        if not self.selected_marker in self.marker_info:
            return
        scale = self._widget.scale_spin_box.value()
        length = self._widget.length_spin_box.value()
        name = "marker" + str(self.marker_id)
        self.marker_id += 1                
        self.addMaker(name, copy.deepcopy(self.marker_info[self.selected_marker].pose), True, length, scale)
        self.selectOnlyOneMarker(name)       
        
    def _deleteSelectedMarker(self):
        if len(self.selected_marker) == 0:
            return
        ret = QMessageBox.warning(self._widget, 
                                  "Delete marker", 
                                  "Do you want to delete " + self.selected_marker,
                                  QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            self.deleteMarker(self.selected_marker)
            self.selected_marker = ""               
                               
    def _clearAllMarkers(self):
        if len(self.marker_info) == 0:
            return
        ret = QMessageBox.warning(self._widget, 
                                  "Delete all marker(s)", 
                                  "Do you want to delete all marker(s)",
                                  QMessageBox.Ok, QMessageBox.Cancel)        
        if ret == QMessageBox.Ok:
            for key in self.marker_info:              
                self.server.erase(key)                               
            self.marker_info.clear()
            self.server.applyChanges()
            self.selected_marker = ""                                     
                
        
    def _enableJoyMessageToggled(self, toggled):            
        if toggled:
            if self._widget.joy0_select_combo_box.count() != 0:
                topic_index = self._widget.joy0_select_combo_box.currentIndex()
                topic = self._widget.joy0_select_combo_box.itemText(topic_index)
                self.joy_subscriber = rospy.Subscriber(topic, sensor_msgs.msg.Joy, self.callbackJoystick)
        else:
            self.joy_subscriber.unregister()
                              
    def _resetRotation(self):
        if len(self.selected_marker) == 0:
            return
        self.marker_info[self.selected_marker].pose.orientation.x = 0
        self.marker_info[self.selected_marker].pose.orientation.y = 0
        self.marker_info[self.selected_marker].pose.orientation.z = 0
        self.marker_info[self.selected_marker].pose.orientation.w = 1
        self.server.setPose(self.selected_marker, self.marker_info[self.selected_marker].pose)
        self.server.applyChanges()
        
    def _planMotion(self):
        if len(self.selected_marker) == 0:
            QMessageBox.warning(self._widget, 
                                "No marker selected", 
                                "Please select target marker",
                                QMessageBox.Ok)
            return
        self.moveToMarker(self.selected_marker)
        
       
    def callbackTrajectoryDone(self, status, result):
        rospy.loginfo("done!")
        rospy.loginfo(status)
        rospy.loginfo(result)
        self.arm_is_active = False
        
    def moveToMarker(self, name):
        req = moveit_msgs.srv.GetPositionIKRequest()        
        req.ik_request.group_name = "manipulator";
        req.ik_request.pose_stamped.header.frame_id = "base_link";
        req.ik_request.pose_stamped.pose = self.marker_info[name].pose;
        req.ik_request.avoid_collisions = True;    
        req.ik_request.robot_state.joint_state = self.current_joint_state;         
                         
                      
        try:
            res = moveit_msgs.srv.GetPositionIKResponse()
            res = rospy.ServiceProxy("compute_ik", moveit_msgs.srv.GetPositionIK).call(req)
            if res.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                #check joint distance
                max_diff = 0;
                for i in range(len(self.current_joint_state.position)):
                    diff = res.solution.joint_state.position[i] - self.current_joint_state.position[i]
                    #print "Joint {} diff: {}".format(i, diff)
                    if abs(diff) > max_diff:
                        max_diff = diff
                    
                min_time = abs(max_diff) / 0.5;  
                print "Max joint diff: {}".format(max_diff)
                print "Min time: {}".format(min_time)
                if min_time < 0.5:
                    min_time = 0.5
                
                
                #prepair target 
                trajectory = trajectory_msgs.msg.JointTrajectory()
                trajectory.joint_names = self.current_joint_state.name
                point = trajectory_msgs.msg.JointTrajectoryPoint()
                point.positions = res.solution.joint_state.position
                point.time_from_start = rospy.Duration(min_time);
                trajectory.points.append(point)
                
                
                goal = control_msgs.msg.FollowJointTrajectoryGoal()
                goal.trajectory = trajectory
                goal.trajectory.header.stamp = rospy.Time.now()
                
                self.arm_is_active = True
                self.action_client.send_goal(goal, self.callbackTrajectoryDone)
                                
            elif res.error_code.val == moveit_msgs.msg.MoveItErrorCodes.NO_IK_SOLUTION:
                rospy.loginfo("no IK solution")
            else:
                rospy.loginfo("error " + str(res.error_code))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        pass
        
    
    def deleteMarker(self, name):
        if name in self.marker_info: 
            del self.marker_info[name]                 
            self.server.erase(name)
            self.server.applyChanges()   

    def refreshTopics(self):
        """
        refresh tree view items

        @raise ROSException
        """
        topic_list = rospy.get_published_topics()
        if topic_list == None:
            raise ROSException("Not even a single topic found published. Check network configuration")
        
        else:  # Topics to show are specified.
            #clear
            self._widget.joy0_select_combo_box.clear()            
            for name, type in topic_list:
                if type == "sensor_msgs/Joy":
                    self._widget.joy0_select_combo_box.addItem(name)
                    rospy.loginfo("Add " + name + " to joystick list")
                    
                
    def callbackJoystick(self, msg):           
        if self.last_button_state[0] != msg.buttons[0]:            
            if msg.buttons[0] == 1:    
                self._widget.enable_translation_global.toggle()
            self.last_button_state[0] = msg.buttons[0]            
            
        if self.last_button_state[1] != msg.buttons[1]:            
            if msg.buttons[1] == 1:    
                self._resetRotation()
            self.last_button_state[1] = msg.buttons[1]          
                     
        if not (self._widget.enable_translation.isChecked() or self._widget.enable_rotation.isChecked()):
            return
        if len(self.selected_marker) == 0:
            return
                    
        
        sum = 0.0
        for val in msg.axes:
            sum += val
        if sum == 0.0:
            return
        
        t_scale = self._widget.joy_translation_scaling_spin_box.value()
        r_scale = self._widget.joy_rotation_scaling_spin_box.value()                                                 
        self.moveMarkerRelatively(self.selected_marker, 
                                  -msg.axes[0]*t_scale, -msg.axes[1]*t_scale, msg.axes[2]*t_scale,
                                  -msg.axes[3]*r_scale, -msg.axes[4]*r_scale, msg.axes[5]*r_scale)
    
    def callbackJointState(self, msg):
        self.current_joint_state = msg
    
    def checkIK(self, pose):
        #check IK
        req = moveit_msgs.srv.GetPositionIKRequest()        
        req.ik_request.group_name = "manipulator";
        req.ik_request.pose_stamped.header.frame_id = "base_link";
        req.ik_request.pose_stamped.pose = pose;
        req.ik_request.avoid_collisions = True;                
                      
        try:
            res = rospy.ServiceProxy("compute_ik", moveit_msgs.srv.GetPositionIK).call(req)
            if res.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                return True
            elif res.error_code.val == moveit_msgs.msg.MoveItErrorCodes.NO_IK_SOLUTION:
                rospy.loginfo("no solution")
            else:
                rospy.loginfo("error " + str(res.error_code))
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return False
    
    def getIKJointPositionsWithSeedState(self, pose):
        #check IK
        pass
        
        
        
    def moveMarkerRelatively(self, name, x, y, z, rx, ry, rz):             
        if not self._widget.enable_tx.isChecked():
            x = 0               
        if not self._widget.enable_ty.isChecked():
            y = 0
        if not self._widget.enable_tz.isChecked():
            z = 0
        if not self._widget.enable_rx.isChecked():
            rx = 0
        if not self._widget.enable_ry.isChecked():
            ry = 0        
        if not self._widget.enable_rz.isChecked():
            rz = 0
            
        pose = copy.deepcopy(self.marker_info[name].pose)
                            
                               
        if self._widget.enable_translation.isChecked():
            if self._widget.enable_translation_global.isChecked():
                pose.position.x += x;
                pose.position.y += y;
                pose.position.z += z;
            else:
                marker_t = tf.transformations.translation_matrix((pose.position.x, pose.position.y, pose.position.z))        
                marker_q = (pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w)                
                marker_T = tf.transformations.concatenate_matrices(marker_t, tf.transformations.quaternion_matrix(marker_q))                                       
                result_T = tf.transformations.concatenate_matrices(marker_T, tf.transformations.translation_matrix((x, y, z)))
                result_t = tf.transformations.translation_from_matrix(result_T)                
                result_q = tf.transformations.quaternion_from_matrix(result_T)      
                     
                pose.position.x = result_t[0]
                pose.position.y = result_t[1]
                pose.position.z = result_t[2]
                pose.orientation.x = result_q[0]
                pose.orientation.y = result_q[1]
                pose.orientation.z = result_q[2]
                pose.orientation.w = result_q[3]                               
        
        if self._widget.enable_rotation.isChecked():
            q = tf.transformations.quaternion_from_euler(rx, ry, rz)
            marker_q = (pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w)
            result_q = tf.transformations.quaternion_multiply(marker_q, q)
            pose.orientation.x = result_q[0]
            pose.orientation.y = result_q[1]
            pose.orientation.z = result_q[2]
            pose.orientation.w = result_q[3]
            
        #check IK
        req = moveit_msgs.srv.GetPositionIKRequest()        
        req.ik_request.group_name = "manipulator";
        req.ik_request.pose_stamped.header.frame_id = "base_link";
        req.ik_request.pose_stamped.pose = pose;
        req.ik_request.avoid_collisions = True;                
               
        if self.checkIK(pose):
            self.marker_info[name].pose = pose                                  
            self.server.setPose(name, pose)
            self.server.applyChanges()
            if self._widget.follow_check_box.isChecked():
                self.moveToMarker(name)
                   
    def processFeedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            if not self.marker_info[feedback.marker_name].selected:                
                rospy.loginfo(s + ": button click" + mp + ".")            
                self.selectOnlyOneMarker(feedback.marker_name)
            
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            rospy.loginfo(s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + ".")
        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            if self.checkIK(feedback.pose):
                self.marker_info[feedback.marker_name].pose = feedback.pose                                  
                if self._widget.follow_check_box.isChecked():
                    self.moveToMarker(feedback.marker_name)
            else:
                self.server.setPose(feedback.marker_name, self.marker_info[feedback.marker_name].pose)
                      
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            #rospy.loginfo(s + ": mouse down" + mp + ".")
            pass
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            #rospy.loginfo(s + ": mouse up" + mp + ".")
            pass
        self.server.applyChanges()    
   
    def addMaker(self, name, pose, selectable, length, scale):
        int_marker = InteractiveMarker()
        int_marker.name = name
        int_marker.description = name
        int_marker.header.frame_id = self.fixed_frame
        int_marker.scale = scale
        int_marker.pose = pose                       
        
        markers = []
        color = std_msgs.msg.ColorRGBA()
        color.r = 0.5
        color.g = 0.5
        color.b = 0.5
        color.a = 0.8 
        markers.append(self.makeBox(int_marker, color))        
        markers.append(self.makeArrow(int_marker, length, color))
        
        if selectable:
            self.makeSelectableControl(int_marker, markers)
        else:
            self.makeFreeMoveControl(int_marker, markers)    
            self.make6DofMarker(int_marker)      
            
        self.marker_info[name] = MarkerInfo(pose, length, scale)
        self.marker_info[name].selected = False
                      
        self.server.insert(int_marker, self.processFeedback)
        self.server.applyChanges()
    

    def makeSelectableControl(self, msg, markers):
        control = InteractiveMarkerControl()
        control.always_visible = True;
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        for i in range(len(markers)):
            control.markers.append(markers[i])                  
        msg.controls.append(control)
        return control
  
    def makeFreeMoveControl(self, msg, markers):
        control = InteractiveMarkerControl()
        control.always_visible = True
        #control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        for i in range(len(markers)):
            control.markers.append(markers[i])
        msg.controls.append(control)        
        return control
    
    def selectMarker(self, name):
        if not self.server.erase(name):
            rospy.logerr("Marker" + name + "did not exist on server")               
        self.addMaker(name, 
                      self.marker_info[name].pose, 
                      False, 
                      self.marker_info[name].length, 
                      self.marker_info[name].scale)     
        self.marker_info[name].selected = True
        self.selected_marker = name
        
    def deselectMarker(self, name):
        if not self.server.erase(name):
            rospy.logerr("Marker" + name + "did not exist on server")        
        self.addMaker(name, 
                      self.marker_info[name].pose, 
                      True, 
                      self.marker_info[name].length, 
                      self.marker_info[name].scale)
        self.marker_info[name].selected = False
        self.selected_marker = ""
        
    def selectOnlyOneMarker(self, name):        
        for key in self.marker_info:            
            if self.marker_info[key].selected:                
                self.deselectMarker(key)
        self.selectMarker(name)
        
        
    def makeBox(self, msg, color):
        marker = Marker()    
        marker.type = Marker.CUBE
        marker.scale.x = msg.scale * 0.45
        marker.scale.y = msg.scale * 0.45
        marker.scale.z = msg.scale * 0.45
        marker.color = color           
        return marker
    
    def makeArrow(self, msg, length, color):
        marker = Marker()    
        marker.type = Marker.ARROW
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = -1;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = length
        marker.scale.y = msg.scale * 0.1
        marker.scale.z = msg.scale * 0.1
        marker.color = color           
        return marker
    
    def makeBoxControl(self, msg):
        control = InteractiveMarkerControl()
        control.always_visible = True
        control.orientation_mode = InteractiveMarkerControl.VIEW_FACING
        control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        control.markers.append(self.makeBox(msg))
        msg.controls.append(control)        
        return control
        
    def make6DofMarker(self, msg):        
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        msg.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        msg.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        msg.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        msg.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        msg.controls.append(control)
    
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        msg.controls.append(control)
            
        
        
        
        
        
        

