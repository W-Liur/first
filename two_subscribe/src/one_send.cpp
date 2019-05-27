#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
int main(int argc,char **argv)
{
ros::init(argc, argv, "one_node");
//不可在同一节点发送
ros::NodeHandle n;
ros::Publisher pub=n.advertise<geometry_msgs::PoseStamped>("pose2",1);
geometry_msgs::PoseStamped msg;
msg.header.frame_id = "pose2";
ros::Rate rate(2);
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
