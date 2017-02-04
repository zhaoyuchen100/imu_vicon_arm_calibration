#!/usr/bin/env python

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin

server = None
menu_handler = MenuHandler()
br = None
#counter = 0

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (-0.395348085615453, 0.222880076997622, 1.01), (0, 0, 0, 1.0), time,  "Upper_arm" ,"base_link")
    br.sendTransform( (0, 2.0664E-05, -0.31315), (0, 0, 0, 1.0), time, "Lower_arm", "Upper_arm")
    br.sendTransform( (-0.25152, -0.0072209, 0.0), (0, 0, 0, 1.0), time, "Hand",  "Lower_arm")
    #counter += 1

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
# TODO
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
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def makeUpperArm( msg ):
    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.mesh_resource = "package://fore_arm_v2.SLDASM/meshes/Shoulder_In_Ex_L.STL"

    return marker

def makeLowerArm( msg ):
    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.mesh_resource = "package://fore_arm_v2.SLDASM/meshes/Lower_arm.STL"

    return marker
def makeHand( msg ):
    marker = Marker()
    marker.type = Marker.MESH_RESOURCE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.mesh_resource = "package://fore_arm_v2.SLDASM/meshes/Wrist_pro_sup_L.STL"

    return marker

def makeControl( msg, frame_id ):
    if frame_id == "Upper_arm":
        #print frame_id
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( makeUpperArm(msg) )
        msg.controls.append( control )
    if frame_id == "Lower_arm":
        #print frame_id
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( makeLowerArm(msg) )
        msg.controls.append( control )
    if frame_id == "Hand":
        #print frame_id
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( makeHand(msg) )
        msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def make6DofMarker( fixed, interaction_mode, position, frame_id, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "6dof" + frame_id
    int_marker.description = "6-DOF Control"

    # insert a component
    makeControl(int_marker,frame_id)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )### they can't registered under the same name!!!



if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()

    # create a timer to update the published transforms
    #rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )


    Upper_arm_position = Point(0,0,0)
    make6DofMarker( False, InteractiveMarkerControl.NONE, Upper_arm_position, "Upper_arm", True)
    Lower_arm_position = Point(0,0,0)
    make6DofMarker( False, InteractiveMarkerControl.NONE, Lower_arm_position,"Lower_arm", True)
    Hand_position = Point(0,0,0)
    make6DofMarker( False, InteractiveMarkerControl.NONE, Hand_position,"Hand", True)

    server.applyChanges()

    marker_pub = rospy.Publisher('visualization_marker', visualization_msgs.msg.Marker, queue_size=10)


    rate = rospy.Rate(30) # rate is 10Hz
    while not rospy.is_shutdown():
####### update marker #####################
    	    int_upper_arm_marker = Marker()
	    int_upper_arm_marker.header.frame_id = "Upper_arm"
	    int_upper_arm_marker.header.stamp = rospy.Time.now()
	    int_upper_arm_marker.ns = "Right_arm"
	    int_upper_arm_marker.id = 0
	    int_upper_arm_marker.type = Marker.MESH_RESOURCE
	    int_upper_arm_marker.pose.position.x = 0
	    int_upper_arm_marker.pose.position.y = 0
	    int_upper_arm_marker.pose.position.z = 0
	    int_upper_arm_marker.pose.orientation.x = 0.0
	    int_upper_arm_marker.pose.orientation.y = -0.70711
	    int_upper_arm_marker.pose.orientation.z = 0.0
	    int_upper_arm_marker.pose.orientation.w = 0.70711
	    int_upper_arm_marker.scale.x = 1
	    int_upper_arm_marker.scale.y = 1
	    int_upper_arm_marker.scale.z = 1
	    int_upper_arm_marker.color.a = 1.0
	    int_upper_arm_marker.color.r = 0.5
	    int_upper_arm_marker.color.g = 0.5
	    int_upper_arm_marker.color.b = 0.5
	    int_upper_arm_marker.mesh_resource = "package://fore_arm_v2.SLDASM/meshes/Shoulder_In_Ex_L.STL";

	    int_lower_arm_marker = Marker()
	    int_lower_arm_marker.header.frame_id = "Lower_arm"
	    int_lower_arm_marker.header.stamp = rospy.Time.now()
	    int_lower_arm_marker.ns = "Right_arm"
	    int_lower_arm_marker.id = 1
	    int_lower_arm_marker.type = Marker.MESH_RESOURCE
	    int_lower_arm_marker.pose.position.x = 0
	    int_lower_arm_marker.pose.position.y = 0
	    int_lower_arm_marker.pose.position.z = 0
	    int_lower_arm_marker.pose.orientation.x = 0.0
	    int_lower_arm_marker.pose.orientation.y = 1.0
	    int_lower_arm_marker.pose.orientation.z = 0.0
	    int_lower_arm_marker.pose.orientation.w = 0.0
	    int_lower_arm_marker.scale.x = 1
	    int_lower_arm_marker.scale.y = 1
	    int_lower_arm_marker.scale.z = 1
	    int_lower_arm_marker.color.a = 1.0
	    int_lower_arm_marker.color.r = 0.5
	    int_lower_arm_marker.color.g = 0.5
	    int_lower_arm_marker.color.b = 0.5
	    int_lower_arm_marker.mesh_resource = "package://fore_arm_v2.SLDASM/meshes/Lower_arm.STL";

	    int_hand_marker = Marker()
	    int_hand_marker.header.frame_id = "Hand"
	    int_hand_marker.header.stamp = rospy.Time.now()
	    int_hand_marker.ns = "Right_arm"
	    int_hand_marker.id = 2
	    int_hand_marker.type = Marker.MESH_RESOURCE
	    int_hand_marker.pose.position.x = 0
	    int_hand_marker.pose.position.y = 0
	    int_hand_marker.pose.position.z = 0
	    int_hand_marker.pose.orientation.x = 0
	    int_hand_marker.pose.orientation.y = 0.70711
	    int_hand_marker.pose.orientation.z = 0.70711
	    int_hand_marker.pose.orientation.w = 0
	    int_hand_marker.scale.x = 1
	    int_hand_marker.scale.y = 1
	    int_hand_marker.scale.z = 1
	    int_hand_marker.color.a = 1.0
	    int_hand_marker.color.r = 0.5
	    int_hand_marker.color.g = 0.5
	    int_hand_marker.color.b = 0.5
	    int_hand_marker.mesh_resource = "package://fore_arm_v2.SLDASM/meshes/Wrist_pro_sup_L.STL";
###########################################
	    marker_pub.publish(int_upper_arm_marker)
	    marker_pub.publish(int_lower_arm_marker)
	    marker_pub.publish(int_hand_marker)

		#rospy.spin()
	    rate.sleep()

