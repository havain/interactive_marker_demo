//
// Created by ht on 2022/5/20.
//


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

geometry_msgs::PoseStamped current_pose;
ros::Publisher init_pose_pub, target_pose_pub, auto_move_pose_pub;

Marker makeBox( InteractiveMarker &msg )
{
    Marker marker;

    marker.type = Marker::ARROW;
    marker.scale.x = msg.scale * 1;
    marker.scale.y = msg.scale * 0.1;
    marker.scale.z = msg.scale * 0.1;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeBox(msg) );
    msg.controls.push_back( control );

    return msg.controls.back();
}

void frameCallback(const ros::TimerEvent&)
{
    static uint32_t counter = 0;

    static tf::TransformBroadcaster br;

    tf::Transform t;

    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter)/140.0) * 2.0));
    t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br.sendTransform(tf::StampedTransform(t, time, "map", "moving_frame"));

    t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter)/140.0, 0.0));
    br.sendTransform(tf::StampedTransform(t, time, "map", "rotating_frame"));

    counter++;
}


void processFeedback(
        const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    current_pose.pose = feedback->pose;
    current_pose.header = feedback->header;
    ROS_INFO_STREAM( "pose changed"
                             << "\nposition = "
                             << feedback->pose.position.x
                             << ", " << feedback->pose.position.y
                             << ", " << feedback->pose.position.z
                             << "\norientation = "
                             << feedback->pose.orientation.w
                             << ", " << feedback->pose.orientation.x
                             << ", " << feedback->pose.orientation.y
                             << ", " << feedback->pose.orientation.z
                             << "\nframe: " << feedback->header.frame_id
                             << " time: " << feedback->header.stamp.sec << "sec, "
                             << feedback->header.stamp.nsec << " nsec" );
}

void setInitPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.pose.pose = current_pose.pose;
    pose.header = current_pose.header;

    init_pose_pub.publish(pose);

    ROS_INFO_STREAM("set inital pose.");
    server->applyChanges();
}



void setTargetPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    geometry_msgs::PoseStamped pose;
    pose = current_pose;
    target_pose_pub.publish(pose);
    ROS_INFO_STREAM("set target pose.");
    server->applyChanges();
}


/*
void setControlPoseCB( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    geometry_msgs::PoseStamped pose;
    pose = current_pose;
    auto_move_pose_pub.publish(pose);
    ROS_INFO_STREAM("set auto control target pose.");
    server->applyChanges();
}
*/

void make6DofMarker( bool fixed, unsigned int interaction_mode, const tf::Vector3& position, bool show_6dof )
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "simple_6dof";
    int_marker.description = "Simple 6-DOF Control";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if ( fixed )
    {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
    {
        std::string mode_text;
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_3D )         mode_text = "MOVE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::ROTATE_3D )       mode_text = "ROTATE_3D";
        if( interaction_mode == visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D )  mode_text = "MOVE_ROTATE_3D";
        int_marker.name += "_" + mode_text;
        int_marker.description = std::string("3D Control") + (show_6dof ? " + 6-DOF controls" : "") + "\n" + mode_text;
    }

    if(show_6dof)
    {
        tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);

        orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
        orien.normalize();
        tf::quaternionTFToMsg(orien, control.orientation);
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);

        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        int_marker.controls.push_back(control);

    }

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    if (interaction_mode != visualization_msgs::InteractiveMarkerControl::NONE)
        menu_handler.apply( *server, int_marker.name );
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_marker_demo");
    ros::NodeHandle n;
    //ros::Publisher init_pose_pub, target_pose_pub, auto_move_pose_pub;


    init_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    target_pose_pub = n.advertise<geometry_msgs::PoseStamped>("web_rviz/goal", 1);
    //auto_move_pose_pub = n.advertise<geometry_msgs::PoseStamped>("control_msgs_simple/goal", 1);


    // create a timer to update the published transforms
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.01), frameCallback);

    server.reset( new interactive_markers::InteractiveMarkerServer("pose_marker","",false) );

    ros::Duration(0.1).sleep();

    menu_handler.insert( "2D Pose Estimate", &setInitPoseCB );
    menu_handler.insert( "2D Target Goal", &setTargetPoseCB );
    //menu_handler.insert( "2D AutoMove Goal", &setControlPoseCB );

    tf::Vector3 position;
    position = tf::Vector3( 0, 0, 0);
    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, position, true );

    server->applyChanges();
    ros::spin();

    server.reset();
}
