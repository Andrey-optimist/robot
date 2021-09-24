#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");
  ros::NodeHandle node;

  tf2_ros::TransformBroadcaster tfb;
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "robot2/odom";

  ros::Publisher robot_vel = node.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 10);

  ros::Rate rate(10.0);
  while (node.ok()){

    try{
      transformStamped = tfBuffer.lookupTransform("robot2/odom", "robot2/base_footprint",ros::Time(0));
    }

    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    robot_vel.publish(vel_msg);

    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);

    rate.sleep();
  } 

};

