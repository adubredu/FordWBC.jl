#include "subpub/subpub.hpp"

Subpub::Subpub(){
    char **d; int a;
    ros::init(a, d, "subpub_node");
    ros::NodeHandle nh;
    subposearray = nh.subscribe<yolo_seg_network::SceneObjectArray>("/state_estimation", 
                    1, &Subpub::pose_array_callback, this);
    ROS_INFO("READY"); 
    ros::spinOnce();
}

void Subpub::pose_array_callback(const yolo_seg_network::SceneObjectArray::ConstPtr& inposes){
    poses.scene_objects = inposes->scene_objects; 
    return;
}

std::vector<std::vector<double>> Subpub::get_poses(){
    ros::spinOnce();
    std::vector<std::vector<double>> out_poses;
    for (auto pose : poses.scene_objects){
        std::vector<double>out_pose;
        // std::cout << pose.pose.position.x << std::endl;
        out_pose.push_back(pose.pose.position.x);
        // out_pose.push_back(poses.header.seq);
        out_pose.push_back(pose.pose.position.y);
        out_pose.push_back(pose.pose.position.z);
        out_pose.push_back(pose.pose.orientation.w);
        out_pose.push_back(pose.pose.orientation.x);
        out_pose.push_back(pose.pose.orientation.y);
        out_pose.push_back(pose.pose.orientation.z);
        out_poses.push_back(out_pose);
    }
    return out_poses;
}