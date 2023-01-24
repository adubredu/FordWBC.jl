#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <lcm/lcm-cpp.hpp>
#include "pose_t.hpp"
#include "posearray_t.hpp"



using namespace std;

class Subpub{
    private:
        geometry_msgs::PoseArray poses;
        ros::NodeHandle nh;
        ros::Subscriber subposearray;
        lcm::LCM lcm;

    public:
        Subpub(ros::NodeHandle* nodehandle);
        void pose_array_callback(const geometry_msgs::PoseArray::ConstPtr& posearray);

};