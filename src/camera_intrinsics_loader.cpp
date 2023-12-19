
#include "../include/calibration_configs_loader/static_info_parser.h"

#include <ros/ros.h>

#include <iostream>

int main(int argc, char** argv){

    ros::init(argc, argv, "camera_info_loader", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if(argc <2){
        printf("need param name! \n");
        return -1;
    }
    std::string cam_info_param_name = argv[1];



    calibration_configs_loader::StaticInfoParser parser(nh, cam_info_param_name, "");

    if( parser.loadCameraIntrinsics()){
        parser.staticPublishCameraIntrinsics();
    }
    else{
        return -1;
    }


    return 0;
}
