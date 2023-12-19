

#include "../include/calibration_configs_loader/static_info_parser.h"

//ros
#include <ros/ros.h>


#include <yaml.h>

// c++
#include <iostream>
#include <thread>


namespace calibration_configs_loader{

    StaticInfoParser::StaticInfoParser(ros::NodeHandle nh, const std::string &camera_intrinsics_param_name,
                                       const std::string &tf_info_param_name) :
            nh_(nh),
            camera_intrinsics_param_name_(camera_intrinsics_param_name),
            tf_info_param_name_(tf_info_param_name){




    }

    bool StaticInfoParser::loadCameraIntrinsics() {

        XmlRpc::XmlRpcValue doc;
        if (!ros::param::has(camera_intrinsics_param_name_) ||
            !ros::param::get(camera_intrinsics_param_name_, doc)) {
            printf("Could not reads  param:%s from  parameter server! \n", camera_intrinsics_param_name_.c_str());
            return false;
        }

        int camera_num = doc.size();
        CameraInfo cam_info;
        for (int i = 0; i < camera_num; ++i) {

            if(!validateXmlRpcCamIntrinsics(doc[i])){
                printf("Could not validate camera intrinsics from %s \n",
                       camera_intrinsics_param_name_.c_str());
                return false;
            }

            readCamIntrinsics(doc[i], cam_info);
            camera_intrinsics_arr.push_back(cam_info);
        }

        return true;
    }

    bool StaticInfoParser::validateXmlRpcCamIntrinsics(XmlRpc::XmlRpcValue cam_intrinsics_data) {

        return cam_intrinsics_data.hasMember("camera_name") &&
               cam_intrinsics_data.hasMember("image_size") &&
               cam_intrinsics_data["image_size"].hasMember("width") &&
               cam_intrinsics_data["image_size"].hasMember("height") &&
               cam_intrinsics_data.hasMember("camera_matrix") &&
               cam_intrinsics_data["camera_matrix"].hasMember("fx") &&
               cam_intrinsics_data["camera_matrix"].hasMember("fy") &&
               cam_intrinsics_data["camera_matrix"].hasMember("cx") &&
               cam_intrinsics_data["camera_matrix"].hasMember("cy") &&
               cam_intrinsics_data.hasMember("distortion") &&
               cam_intrinsics_data["distortion"].hasMember("distortion_model") &&
               cam_intrinsics_data["distortion"].hasMember("distortion_coefficients") &&
               cam_intrinsics_data["distortion"]["distortion_coefficients"].hasMember("k1") &&
               cam_intrinsics_data["distortion"]["distortion_coefficients"].hasMember("k2") &&
               cam_intrinsics_data["distortion"]["distortion_coefficients"].hasMember("t1") &&
               cam_intrinsics_data["distortion"]["distortion_coefficients"].hasMember("t2") &&
               cam_intrinsics_data["distortion"]["distortion_coefficients"].hasMember("k3");
    }

    void StaticInfoParser::readCamIntrinsics(XmlRpc::XmlRpcValue cam_intrinsics_data, CameraInfo &cam_info) {
        cam_info.camera_name = (std::string) cam_intrinsics_data["camera_name"];

        cam_info.image_width = (int) cam_intrinsics_data["image_size"]["width"];
        cam_info.image_height = (int) cam_intrinsics_data["image_size"]["height"];

        cam_info.fx = (double) cam_intrinsics_data["camera_matrix"]["fx"];
        cam_info.fy = (double) cam_intrinsics_data["camera_matrix"]["fy"];
        cam_info.cx = (double) cam_intrinsics_data["camera_matrix"]["cx"];
        cam_info.cy = (double) cam_intrinsics_data["camera_matrix"]["cy"];

        cam_info.distortion_model = (std::string) cam_intrinsics_data["distortion"]["distortion_model"];
        cam_info.k1 = (double) cam_intrinsics_data["distortion"]["distortion_coefficients"]["k1"];
        cam_info.k2 = (double) cam_intrinsics_data["distortion"]["distortion_coefficients"]["k2"];
        cam_info.t1 = (double) cam_intrinsics_data["distortion"]["distortion_coefficients"]["t1"];
        cam_info.t2 = (double) cam_intrinsics_data["distortion"]["distortion_coefficients"]["t1"];
        cam_info.k3 = (double) cam_intrinsics_data["distortion"]["distortion_coefficients"]["k3"];

        return;
    }

    bool StaticInfoParser::loadTf() {


        XmlRpc::XmlRpcValue doc;
        if (!ros::param::has(tf_info_param_name_) || !ros::param::get(tf_info_param_name_, doc)) {
            printf("Could not reads  param:%s from  parameter server! \n", tf_info_param_name_.c_str());
            return false;
        }

        Transform tf_info;
        int tf_num = doc.size();
        for (int i = 0; i < tf_num; ++i) {
            if(!validateXmlRpcTF(doc[i])){
                printf("Could not validate tf from %s \n", tf_info_param_name_.c_str());
                return false;
            }
            readTF(doc[i], tf_info);
            tf_arr.push_back(tf_info);
        }

        return true;
    }

    bool StaticInfoParser::validateXmlRpcTF(XmlRpc::XmlRpcValue tf_data) {
        return tf_data.hasMember("sensor_name") &&
               tf_data.hasMember("reference_sensor_name") &&
               tf_data.hasMember("sensor_type") &&
               tf_data.hasMember("transform") &&
                 tf_data["transform"].hasMember("translation") &&
                   tf_data["transform"]["translation"].hasMember("x") &&
                   tf_data["transform"]["translation"].hasMember("y") &&
                   tf_data["transform"]["translation"].hasMember("z") &&
                 tf_data["transform"].hasMember("rotation") &&
                   tf_data["transform"]["rotation"].hasMember("x") &&
                   tf_data["transform"]["rotation"].hasMember("y") &&
                   tf_data["transform"]["rotation"].hasMember("z") &&
                   tf_data["transform"]["rotation"].hasMember("w");
    }

    void StaticInfoParser::readTF(XmlRpc::XmlRpcValue tf_data, Transform &tf_info) {

        tf_info.sensor_name = (std::string) tf_data["sensor_name"];
        tf_info.reference_sensor_name  = (std::string) tf_data["reference_sensor_name"];
        tf_info.sensor_type = (std::string) tf_data["sensor_type"];

        tf_info.translation_x = (double) tf_data["transform"]["translation"]["x"];
        tf_info.translation_y = (double) tf_data["transform"]["translation"]["y"];
        tf_info.translation_z = (double) tf_data["transform"]["translation"]["z"];

        tf_info.rotation_x = (double) tf_data["transform"]["rotation"]["x"];
        tf_info.rotation_y = (double) tf_data["transform"]["rotation"]["y"];
        tf_info.rotation_z = (double) tf_data["transform"]["rotation"]["z"];
        tf_info.rotation_w = (double) tf_data["transform"]["rotation"]["w"];

        return;
    }

    void StaticInfoParser::staticPublishTf() {

        tf_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
                "transform_imu", 10, false);

       int duration = 10;  // unit ms


        int tf_num = tf_arr.size();
        std::thread worker_list[tf_num];
        for (int entry = 0; entry < tf_num; ++entry) {
            worker_list[entry] = std::thread(&StaticInfoParser::staticPublisheOneTf,
                                                 this, duration, entry);
        }

        for(uint entry = 0; entry < tf_num; ++entry){
            worker_list[entry].join();
        }

    }


    void StaticInfoParser::staticPublisheOneTf(int period, int entry) {
        ros::Duration sleeper(period/1000.0);  // period  unit ms


        Transform tf_info = tf_arr[entry];
        TransformSender tf_sender(tf_info.translation_x, tf_info.translation_y, tf_info.translation_z,
                                  tf_info.rotation_x, tf_info.rotation_y, tf_info.rotation_z, tf_info.rotation_w,
                                  ros::Time()+ sleeper, tf_info.reference_sensor_name, tf_info.sensor_name);

        while(ros::ok()){

            ros::Time time = ros::Time::now();

            tf_sender.send(time + sleeper);

            sleeper.sleep();
        }
    }

    void StaticInfoParser::staticPublishCameraIntrinsics() {

        int duration = 10;


        int cam_num = camera_intrinsics_arr.size();

        std::thread worker_list[cam_num];
        for (int entry = 0; entry < cam_num; ++entry) {
            worker_list[entry] = std::thread(&StaticInfoParser::staticPublishOneIntrinsics,
                                             this, duration, entry);
        }

        for(uint entry = 0; entry < cam_num; ++entry){
            worker_list[entry].join();
        }

    }


    void StaticInfoParser::staticPublishOneIntrinsics(int period, int entry) {
        ros::Duration sleeper(period / 1000.0);  // sleeper = 10ms



        CameraInfo cam_info = camera_intrinsics_arr[entry];
        CameraInfoSender cam_info_sender(cam_info.camera_name,
                                         cam_info.image_width, cam_info.image_height,
                                         cam_info.fx, cam_info.fy, cam_info.cx, cam_info.cy,
                                         cam_info.distortion_model,
                                         cam_info.k1, cam_info.k2,
                                         cam_info.t1, cam_info.t2, cam_info.k3,
                                         ros::Time() + sleeper);

        while(ros::ok()){

            ros::Time time = ros::Time::now();

            cam_info_sender.send(time + sleeper);

        }

        sleeper.sleep();
    }


    bool StaticInfoParser::getCameraInfo(std::string cam_name, CameraInfo *cam_info) {

        for(auto entry : camera_intrinsics_arr){
            if(entry.camera_name == cam_name){
                *cam_info = entry;
                return true;
            }
        }

        return false;
    }




    TransformSender::TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw,
                                     ros::Time time, const std::string &frame_id, const std::string &child_frame_id) :
            transform_(tf::Transform(tf::Quaternion(qx,qy,qz,qw), tf::Vector3(x,y,z)),
                   time, frame_id, child_frame_id){

    }

    void TransformSender::send(ros::Time time) {
        transform_.stamp_ = time;
        broadcaster.sendTransform(transform_);
    }

    TransformSender::~TransformSender() {
    }

    CameraInfoSender::CameraInfoSender(std::string camera_name, int width, int height,
                                       double fx, double fy, double cx, double cy,
                                       std::string distortion_model,
                                       double k1, double k2, double t1, double t2, double k3, ros::Time time) {
        ros::NodeHandle node;
        cam_info_pub = node.advertise<sensor_msgs::CameraInfo>(camera_name+"/camera_info", 1000);


        cam_info_.header.frame_id = camera_name;
        cam_info_.header.stamp = time;

        cam_info_.width = width;
        cam_info_.height = height;

        cam_info_.K[0] = fx;
        cam_info_.K[4] = fy;
        cam_info_.K[2] = cx;
        cam_info_.K[5] = cy;

        cam_info_.distortion_model = distortion_model;
        if(cam_info_.distortion_model == "plumb_bob"){
            const size_t kNumDistortionParams = 5;
            cam_info_.D.resize(5);
            cam_info_.D[0] = k1;
            cam_info_.D[1] = k2;
            cam_info_.D[2] = t1;
            cam_info_.D[3] = t2;
            cam_info_.D[4] = k3;
        }else{
            printf("just plumb_bob distortion model is supported!");
            exit(-1);
        }



    }

    void CameraInfoSender::send(ros::Time time) {
        cam_info_.header.stamp = time;
        cam_info_pub.publish(cam_info_);
    }
} // calibration_configs_loader
