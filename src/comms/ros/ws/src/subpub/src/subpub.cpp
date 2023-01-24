#include "subpub/subpub.hpp"

Subpub::Subpub(ros::NodeHandle* nodehandle):nh(*nodehandle){
    subposearray = nh.subscribe<geometry_msgs::PoseArray>("/tag_poses", 
                    1, &Subpub::pose_array_callback, this);
    ROS_INFO("READY");
}

void Subpub::pose_array_callback(const geometry_msgs::PoseArray::ConstPtr& inposes){
    posearray_t lcmposes;
    for (auto pose : inposes->poses){
        pose_t lcmpose;
        lcmpose.timestamp = inposes->header.stamp.nsec;
        lcmpose.x = pose.position.x;
        lcmpose.y = pose.position.y; 
        lcmpose.z = pose.position.z;
        lcmpose.qw = pose.orientation.w;
        lcmpose.qx = pose.orientation.x;
        lcmpose.qy = pose.orientation.y;
        lcmpose.qz = pose.orientation.z;
        lcmposes.poses.push_back(lcmpose);
    }
    lcmposes.timestamp = inposes->header.seq;
    lcmposes.num_poses = lcmposes.poses.size();
    lcm.publish("tag_poses", &lcmposes);


    ROS_INFO("Sequence: %d", inposes->header.seq);

    return;
}

