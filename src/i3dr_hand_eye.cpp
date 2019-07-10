#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/aruco/dictionary.hpp"
#include "opencv2/aruco.hpp"

int STATE_CAMERA = 0;
int STATE_ARM = 1;
int state = STATE_CAMERA;

bool IMAGE_READY = false;
bool MARKER_READY = false;
bool CAL_TF_READY = false;

std::string camera_base_frame, camera_frame, world_frame, arm_tooltip_frame, arm_base_frame;
bool camera_mounted_on_robot, publish_aruco_tf = false;

sensor_msgs::ImageConstPtr MSG_LEFT_IMAGE;
sensor_msgs::ImageConstPtr MSG_RIGHT_IMAGE;
sensor_msgs::CameraInfoConstPtr MSG_CAMERA_INFO_LEFT;
sensor_msgs::CameraInfoConstPtr MSG_CAMERA_INFO_RIGHT;

tf::Transform CAL_TF;
tf::Transform ARUCO_CAMERA_TF;
tf::Transform USER_DEFINED_ROBOT_BASE_ARUCO_MARKER_TF;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
    policy_t;

void cameraInfo_to_KDRP(bool rectified_image, const sensor_msgs::CameraInfoConstPtr msg_camera_info, cv::Mat &K, cv::Mat &D, cv::Mat &R, cv::Mat &P)
{
    if (rectified_image)
    {
        K = cv::Mat(3, 3, CV_64FC1);
        K.at<double>(0, 0) = msg_camera_info->P[0];
        K.at<double>(0, 1) = msg_camera_info->P[1];
        K.at<double>(0, 2) = msg_camera_info->P[2];
        K.at<double>(1, 0) = msg_camera_info->P[4];
        K.at<double>(1, 1) = msg_camera_info->P[5];
        K.at<double>(1, 2) = msg_camera_info->P[6];
        K.at<double>(2, 0) = msg_camera_info->P[8];
        K.at<double>(2, 1) = msg_camera_info->P[9];
        K.at<double>(2, 2) = msg_camera_info->P[10];

        D = cv::Mat(4, 1, CV_64FC1);
        for (int i = 0; i < 4; i++)
            D.at<double>(i, 0) = 0;
    }
    else
    {
        K = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->K.data());
        D = cv::Mat(1, 5, CV_64FC1, (void *)msg_camera_info->D.data());
        R = cv::Mat(3, 3, CV_64FC1, (void *)msg_camera_info->R.data());
        P = cv::Mat(3, 4, CV_64FC1, (void *)msg_camera_info->P.data());
    }
}

void createArucoMarkers()
{
    cv::Mat outputMarker;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_5X5_50);
    for (int i = 0; i < 50; i++)
    {
        cv::aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
        std::ostringstream convert;
        std::string imageName = "4x4Marker_";
        convert << imageName << i << ".jpg";
        imwrite(convert.str(), outputMarker);
    }
}

tf::StampedTransform getTf(std::string tf_a, std::string tf_b)
{
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;
    try
    {
        tf_listener.waitForTransform(tf_a, tf_b, ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform(tf_a, tf_b, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return (transform);
}

bool findAruco(cv::Mat frame, cv::Mat camera_matrix, cv::Mat camera_distortion, tf::Transform &cam_aruco_tf)
{
    bool valid_aruco;
    cv::aruco::DetectorParameters parameters;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCansidates;

    //cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_ARUCO_ORIGINAL);
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);
    float aruco_marker_length = 0.045;

    std::vector<cv::Vec3d> rotationVectors, translationVectors;

    cv::Mat Rotationmatrix;
    cv::Mat WorldPoints1(3, 1, CV_64FC1);
    cv::Mat WorldPoints2(3, 1, CV_64FC1);
    cv::Mat WorldPoints3(3, 1, CV_64FC1);
    cv::Mat WorldPoints4(3, 1, CV_64FC1);

    cv::Mat ImagePoints1(3, 1, CV_64FC1);
    cv::Mat ImagePoints2(3, 1, CV_64FC1);
    cv::Mat ImagePoints3(3, 1, CV_64FC1);
    cv::Mat ImagePoints4(3, 1, CV_64FC1);
    cv::Mat translationMatrix(3, 1, CV_64FC1);

    //ROS_INFO("Detecting markers...");
    cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    if (markerCorners.size() > 0)
    {
        //cv::aruco::estimatePoseSingleMarkers(markerCorners, aruco_marker_length, camera_matrix, camera_distortion, rotationVectors, translationVectors);

        if (markerIds.size() > 0)
        {
            //ROS_INFO("Creating grid...");
            cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(4, 3, 0.045, 0.0095, markerDictionary, 1);
            //ROS_INFO("Getting pose...");
            cv::Vec3d rvec, tvec;
            int valid = cv::aruco::estimatePoseBoard(markerCorners, markerIds, board, camera_matrix, camera_distortion, rvec, tvec);
            if (valid > 0)
            {
                cv::aruco::refineDetectedMarkers(frame, board, markerCorners, markerIds, rejectedCansidates);
                //ROS_INFO("Drawing markers...");
                cv::aruco::drawAxis(frame, camera_matrix, camera_distortion, rvec, tvec, aruco_marker_length);

                //ROS_INFO("Getting markers tf...");

                bool rodrigues = true;

                tf::Quaternion q;
                // convert to quaternion
                // http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToQuaternion/index.htm
                double angle = cv::norm(rvec);
                cv::Vec3d axis = rvec / angle;
                q.setRotation(tf::Vector3(axis[0], axis[1], axis[2]), angle);

                tf::Vector3 marker_tran = tf::Vector3(tvec.val[0], tvec.val[1], tvec.val[2]);
                cam_aruco_tf = tf::Transform(q, marker_tran);
                //std::cout << marker_tran.getX() << "," << marker_tran.getY() << "," << marker_tran.getZ() << std::endl;

                //ROS_INFO("Done.");

                valid_aruco = true;
            }
            else
            {
                valid_aruco = false;
                ROS_INFO("No Aruco found");
            }
        }
        else
        {
            valid_aruco = false;
            ROS_INFO("No Aruco found");
        }
    }
    else
    {
        valid_aruco = false;
        ROS_INFO("No Aruco found");
    }

    return valid_aruco;
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg_image_left, const sensor_msgs::ImageConstPtr &msg_image_right, const sensor_msgs::CameraInfoConstPtr &msg_camera_info_left, const sensor_msgs::CameraInfoConstPtr &msg_camera_info_right)
{
    MSG_LEFT_IMAGE = msg_image_left;
    MSG_CAMERA_INFO_LEFT = msg_camera_info_left;
    MSG_RIGHT_IMAGE = msg_image_right;
    MSG_CAMERA_INFO_RIGHT = msg_camera_info_right;
    IMAGE_READY = true;
}

bool trig_aruco_robot(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    ROS_INFO("aruco robot alignment triggered");
    //get transform from aruco marker to tooltip
    tf::StampedTransform arcuo_to_tooltip_tf = getTf(arm_tooltip_frame,arm_base_frame);

    tf::Transform rotate_world_tf;
    tf::Quaternion q;
    q.setRPY(3.14, 0, -1.57);
    rotate_world_tf.setRotation(q);

    tf::Transform rot_world_tf = rotate_world_tf * arcuo_to_tooltip_tf;

    //transform world->robot_base so that aruco marker == tooltip
    static tf::TransformBroadcaster tf_br;
    tf_br.sendTransform(tf::StampedTransform(rot_world_tf, ros::Time::now(), world_frame, arm_base_frame));
    return (true);
}

void calc_camera_tf()
{
    

    if (camera_mounted_on_robot)
    {
        //find transform from camera_left to camera_base
        tf::StampedTransform camera_left_to_base_tf = getTf(camera_base_frame, camera_frame);

        //find transform from aruco_marker to camera_base
        tf::Transform aruco_marker_to_camera_base_tf;
        aruco_marker_to_camera_base_tf = camera_left_to_base_tf * ARUCO_CAMERA_TF;
        aruco_marker_to_camera_base_tf = aruco_marker_to_camera_base_tf.inverse();
        //find transform from tool to robot base
        tf::StampedTransform tool_to_robot_base_tf = getTf(arm_tooltip_frame, arm_base_frame);
        std::cout << "Rt->Rb " << tool_to_robot_base_tf.getOrigin().getX() << "," << tool_to_robot_base_tf.getOrigin().getY() << "," << tool_to_robot_base_tf.getOrigin().getZ() << std::endl;

        //get transform from robot base to aruco marker (from user input)
        tf::Transform robot_base_to_aruco_marker_tf = USER_DEFINED_ROBOT_BASE_ARUCO_MARKER_TF;
        std::cout << "Rb->A " << robot_base_to_aruco_marker_tf.getOrigin().getX() << "," << robot_base_to_aruco_marker_tf.getOrigin().getY() << "," << robot_base_to_aruco_marker_tf.getOrigin().getZ() << std::endl;

        std::cout << "A->Cb " << aruco_marker_to_camera_base_tf.getOrigin().getX() << "," << aruco_marker_to_camera_base_tf.getOrigin().getY() << "," << aruco_marker_to_camera_base_tf.getOrigin().getZ() << std::endl;

        CAL_TF = tool_to_robot_base_tf * robot_base_to_aruco_marker_tf * aruco_marker_to_camera_base_tf;

        std::cout << "Rt->Cb T: " << CAL_TF.getOrigin().getX() << "," << CAL_TF.getOrigin().getY() << "," << CAL_TF.getOrigin().getZ() << std::endl;
        std::cout << "Rt->Cb R: " << CAL_TF.getRotation().getX() << "," << CAL_TF.getRotation().getY() << "," << CAL_TF.getRotation().getZ() << "," << CAL_TF.getRotation().getW() << std::endl;
    }
    else
    {
        tf::StampedTransform camera_left_to_base_tf = getTf(camera_frame,camera_base_frame);
        tf::Transform aruco_marker_to_camera_base_tf;
        aruco_marker_to_camera_base_tf = camera_left_to_base_tf * ARUCO_CAMERA_TF;
        aruco_marker_to_camera_base_tf = aruco_marker_to_camera_base_tf.inverse();
        CAL_TF = aruco_marker_to_camera_base_tf;
    }
    CAL_TF_READY = true;
}

void aruco_camera()
{
    try
    {
        //ROS_INFO("loading image...");
        cv_bridge::CvImagePtr left_image, right_image;
        left_image = cv_bridge::toCvCopy(MSG_LEFT_IMAGE, "mono8");
        right_image = cv_bridge::toCvCopy(MSG_RIGHT_IMAGE, "mono8");
        cv::Mat Kl, Dl, Rl, Pl, Kr, Dr, Rr, Pr;
        //ROS_INFO("extracting info...");
        cameraInfo_to_KDRP(true, MSG_CAMERA_INFO_LEFT, Kl, Dl, Rl, Pl);
        cameraInfo_to_KDRP(true, MSG_CAMERA_INFO_RIGHT, Kr, Dr, Rr, Pr);
        std::string camera_frame_left = camera_frame;

        //ROS_INFO("detecting aruco...");
        //find transform from aruco to camera left
        tf::Transform camL_aruco_tf, camR_aruco_tf;
        bool aruco_found_l = findAruco(left_image->image, Kl, Dl, camL_aruco_tf);
        bool aruco_found_r = findAruco(right_image->image, Kr, Dr, camR_aruco_tf);

        cv::Mat frame_resize_l, frame_resize_r;
        cv::resize(left_image->image, frame_resize_l, cv::Size(960, 540));
        cv::resize(right_image->image, frame_resize_r, cv::Size(960, 540));
        cv::imshow("i3dr_hand_eye_l", frame_resize_l);
        cv::imshow("i3dr_hand_eye_r", frame_resize_r);
        cv::waitKey(1);

        if (aruco_found_l && aruco_found_r)
        {
            ARUCO_CAMERA_TF = camL_aruco_tf;
            MARKER_READY = true;
        }
        else
        {
            MARKER_READY = false;
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("exception %s", e.what());
    }
}

bool trig_aruco_camera(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    if (IMAGE_READY)
    {
        ROS_INFO("aruco camera alignment triggered");
        if (MARKER_READY)
        {
            calc_camera_tf();
        }
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "i3dr_hand_eye");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string camera_namespace;

    bool save_current_tf = false;

    //Get parameters
    if (p_nh.getParam("camera_namespace", camera_namespace))
    {
        ROS_INFO("camera_namespace: %s", camera_namespace.c_str());
    }
    if (p_nh.getParam("camera_base_frame", camera_base_frame))
    {
        ROS_INFO("camera_base_frame: %s", camera_base_frame.c_str());
    }
    if (p_nh.getParam("camera_frame", camera_frame))
    {
        ROS_INFO("camera_frame: %s", camera_frame.c_str());
    }
    if (p_nh.getParam("world_frame", world_frame))
    {
        ROS_INFO("world_frame: %s", world_frame.c_str());
    }
    if (p_nh.getParam("arm_tooltip_frame", arm_tooltip_frame))
    {
        ROS_INFO("arm_tooltip_frame: %s", arm_tooltip_frame.c_str());
    }
    if (p_nh.getParam("arm_base_frame", arm_base_frame))
    {
        ROS_INFO("arm_base_frame: %s", arm_base_frame.c_str());
    }
    if (p_nh.getParam("camera_mounted_on_robot", camera_mounted_on_robot))
    {
        if (camera_mounted_on_robot)
        {
            ROS_INFO("camera_mounted_on_robot: TRUE");
        }
        else
        {
            ROS_INFO("camera_mounted_on_robot: FALSE");
        }
    }
    if (p_nh.getParam("publish_aruco_tf", publish_aruco_tf))
    {
        if (publish_aruco_tf)
        {
            ROS_INFO("publish_aruco_tf: TRUE");
        }
        else
        {
            ROS_INFO("publish_aruco_tf: FALSE");
        }
    }

    cv::namedWindow("i3dr_hand_eye_r");
    cv::startWindowThread();

    cv::namedWindow("i3dr_hand_eye_l");
    cv::startWindowThread();

    tf::Quaternion q(0, 0, 0.0087265, -0.9999619);
    tf::Vector3 t(0.4, 0.024752, -0.20641);
    USER_DEFINED_ROBOT_BASE_ARUCO_MARKER_TF.setOrigin(t);
    USER_DEFINED_ROBOT_BASE_ARUCO_MARKER_TF.setRotation(q);

    //TODO: add loading previous calibration from yaml
    //TODO: add saving calibration to yaml

    ros::ServiceServer srv_aruco_robot = nh.advertiseService("trig_aruco_robot", trig_aruco_robot);
    ros::ServiceServer srv_aruco_cam = nh.advertiseService("trig_aruco_camera", trig_aruco_camera);

    // Subscribers creation.
    message_filters::Subscriber<sensor_msgs::Image> sub_image_l(nh, camera_namespace + "/left/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_image_r(nh, camera_namespace + "/right/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_l(nh, camera_namespace + "/left/camera_info", 1);
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_r(nh, camera_namespace + "/right/camera_info", 1);

    // Message filter creation.
    message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_image_l, sub_image_r, sub_camera_info_l, sub_camera_info_r);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3, _4));

    static tf::TransformBroadcaster tf_br;

    while (ros::ok())
    {

        if (IMAGE_READY)
        {
            aruco_camera();
            if (MARKER_READY)
            {
                if (publish_aruco_tf)
                {
                    static tf::TransformBroadcaster tf_br;
                    tf_br.sendTransform(tf::StampedTransform(ARUCO_CAMERA_TF, ros::Time::now(), camera_frame, "aruco_marker"));
                    tf_br.sendTransform(tf::StampedTransform(USER_DEFINED_ROBOT_BASE_ARUCO_MARKER_TF, ros::Time::now(), arm_base_frame, "aruco_marker_world"));
                }
            }
            if (CAL_TF_READY)
            {
                if (camera_mounted_on_robot)
                {
                    tf_br.sendTransform(tf::StampedTransform(CAL_TF, ros::Time::now(), arm_tooltip_frame, camera_base_frame));
                }
                else
                {
                    tf_br.sendTransform(tf::StampedTransform(CAL_TF, ros::Time::now(), world_frame, camera_base_frame));
                }
            }
        }

        ros::spinOnce();
    }

    cv::destroyWindow("i3dr_hand_eye");
    return 0;
}