//
// the class is creat to parse
// camera intrinsics and tf information in yaml format file.
//

#ifndef CALIBRATION_CONFIGS_LOADER_STATIC_INFO_PARSER_H
#define CALIBRATION_CONFIGS_LOADER_STATIC_INFO_PARSER_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>


namespace calibration_configs_loader{

    class TransformSender{
    public:
        TransformSender(double x, double y, double z, double qx, double qy, double qz, double qw,
                        ros::Time time, const std::string& frame_id, const std::string& child_frame_id);

        ~TransformSender();


        void send(ros::Time time);


    public:
        tf::TransformBroadcaster broadcaster;

    private:
        tf::StampedTransform transform_;


    };



    class CameraInfoSender{
    public:
        CameraInfoSender(std::string camera_name, int width, int height,
                         double fx, double fy, double cx, double cy,
                         std::string distortion_model,
                         double k1, double k2, double t1, double t2, double k3, ros::Time time);
        void send(ros::Time time);

    public:
        ros::Publisher cam_info_pub;
    private:
        sensor_msgs::CameraInfo cam_info_;
    };


    struct CameraInfo{
        std::string camera_name;
        int image_width, image_height;
        double fx, fy, cx, cy;
        std::string distortion_model;
        double k1, k2, t1, t2, k3;
        CameraInfo(std::string name="", int width=0, int height=0,
                   double fx=0, double fy=0, double cx=0, double cy=0,
                   std::string distortion_model="",
                   double k1=0, double k2=0, double p1=0, double p2=0, double k3=0) :
                camera_name(name), image_width(width), image_height(height),
                fx(fx), fy(fy), cx(cx), cy(cy),
                distortion_model(distortion_model),
                k1(k1), k2(k2), t1(t1), t2(t2), k3(k3){

        }

    };

    struct Transform{
        std::string sensor_name, reference_sensor_name;
        std::string sensor_type;
        double translation_x, translation_y, translation_z;
        double rotation_x, rotation_y, rotation_z, rotation_w;
        Transform(std::string sensor="sensor", std::string ref="reference",
                  double x=0, double y=0, double z=0,
                  double quat_x=0, double quat_y=0, double quat_z=0, double quat_w=0):
                sensor_name(sensor), reference_sensor_name(ref),
                translation_x(x), translation_y(y), translation_z(z),
                rotation_x(quat_x), rotation_y(quat_y), rotation_z(quat_z), rotation_w(quat_w){

        }
    };


    class StaticInfoParser {
    public:
        StaticInfoParser(ros::NodeHandle nh, const std::string &camera_intrinsics_param_name,
                         const std::string &tf_info_param_name);

        bool loadCameraIntrinsics();
        void staticPublishCameraIntrinsics();
        void staticPublishOneIntrinsics(int period, int entry);

        bool loadTf();
        void staticPublishTf();
        void staticPublisheOneTf(int period, int entry);

        bool getCameraInfo(std::string cam_name, CameraInfo *cam_info);

    private:
        bool validateXmlRpcCamIntrinsics(XmlRpc::XmlRpcValue cam_intrinsics_data);
        void readCamIntrinsics(XmlRpc::XmlRpcValue cam_intrinsics_data, CameraInfo& cam_info);

        bool validateXmlRpcTF(XmlRpc::XmlRpcValue tf_data);
        void readTF(XmlRpc::XmlRpcValue tf_data, Transform& tf_info);

    private:
            const std::string camera_intrinsics_param_name_;
            const std::string tf_info_param_name_;
            ros::Publisher tf_pub_;

            ros::NodeHandle nh_;
    public:
            std::vector<CameraInfo> camera_intrinsics_arr;
            std::vector<Transform> tf_arr;

    };

} // namespace calibration_configs_loader



#endif //CALIBRATION_CONFIGS_LOADER_STATIC_INFO_PARSER_H
