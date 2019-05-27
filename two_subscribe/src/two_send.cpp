#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
int main(int argc,char **argv)
{
ros::init(argc, argv, "two_node");
//不可在同名节点发送
ros::NodeHandle nh;
ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("pose1",1);
geometry_msgs::PoseStamped msg;
msg.header.frame_id = "pose1";
ros::Rate rate(1);
int i=0;
while(ros::ok())
{
msg.header.stamp=ros::Time::now();
msg.pose.position.x=i++;
pub.publish(msg);
ros::spinOnce();
rate.sleep();
}
return 0;
}
