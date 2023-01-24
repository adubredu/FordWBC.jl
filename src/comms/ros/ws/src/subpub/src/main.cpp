#include <iostream>
#include "subpub/subpub.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "subpub_node");
    ros::NodeHandle nh;
    Subpub Subpub(&nh);

    ros::spin();
    return EXIT_SUCCESS;
}