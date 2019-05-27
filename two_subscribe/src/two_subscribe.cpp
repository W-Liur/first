#include <message_filters/subscriber.h>
    #include <message_filters/time_synchronizer.h>
    #include <sensor_msgs/Image.h>
   #include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/sync_policies/approximate_time.h>
    //必须有时间戳
//不同步的消息会被覆盖
    void callback(const geometry_msgs::PoseStampedConstPtr& image, const geometry_msgs::PoseStampedConstPtr& cam_info)
   {
     ROS_INFO_STREAM("pose1"<<image->pose.position.x<<"pose1"<<cam_info->pose.position.x);
    // ROS_INFO_STREAM("1");
   }
   typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> sync_policy_classification;
   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "vision_node");
   
     ros::NodeHandle nh;
   
     message_filters::Subscriber<geometry_msgs::PoseStamped> image_sub(nh, "pose1", 1);
     message_filters::Subscriber<geometry_msgs::PoseStamped> info_sub(nh, "pose2", 1);
     //message_filters::TimeSynchronizer<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> sync(image_sub, info_sub, 10);
message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(10), image_sub, info_sub);
ROS_INFO_STREAM("1");
     sync.registerCallback(boost::bind(&callback, _1, _2));
ROS_INFO_STREAM("2");
    ros::spin();
   
     return 0;
   }

