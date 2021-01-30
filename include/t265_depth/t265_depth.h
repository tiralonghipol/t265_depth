#pragma once

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "opencv2/ximgproc.hpp"
#include <image_geometry/stereo_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

using namespace cv;
using namespace cv::ximgproc;
using namespace std;

namespace t265_depth
{

    // stereo parameters
    constexpr int kPreFilterCap = 11;
    constexpr int kSADWindowSize = 11;
    constexpr int kMinDisparity = 0;
    constexpr int kNumDisparities = 64;
    constexpr int kUniquenessRatio = 0;
    constexpr int kSpeckleRange = 3;
    constexpr int kSpeckleWindowSize = 500;
    // bm parameters
    constexpr int kTextureThreshold = 0;
    const std::string kPreFilterType = "xsobel";
    constexpr int kPreFilterSize = 3;
    // sgbm parameters
    constexpr bool kUseSGBM = false;
    constexpr int kP1 = 120;
    constexpr int kP2 = 240;
    constexpr int kDisp12MaxDiff = -1;
    constexpr bool kUseHHMode = false;
    constexpr int kSGBMMode = 0;

    constexpr bool kDoMedianBlur = true;
    constexpr bool kEnableDynReconf = false;

    class t265Depth
    {
    public:
        t265Depth(ros::NodeHandle &node, ros::NodeHandle &private_node);
        ~t265Depth();

        void initializeRectificationMapping(std::string param_file_path);
        void publishCameraInfo(cv::Mat K1, cv::Mat K2,
                               cv::Mat P1, cv::Mat P2,
                               cv::Mat R1, cv::Mat R2);
        void elaborateImages(const std_msgs::Header &header_msg);
        void computePointcloud(const cv::Mat &input_disparity,
                               sensor_msgs::PointCloud2 &pointcloud);

    private:
        void syncCallback(const sensor_msgs::Image::ConstPtr &image_msg_left,
                          const sensor_msgs::Image::ConstPtr &image_msg_right);

        image_transport::ImageTransport it_;
        image_transport::Subscriber sub_img_left_;
        image_transport::Subscriber sub_img_right_;
        image_transport::Publisher pub_img_left_rect_;
        image_transport::Publisher pub_img_right_rect_;
        image_transport::Publisher pub_disparity_;

        ros::Publisher pub_camera_info_left_;
        ros::Publisher pub_camera_info_right_;
        ros::Publisher pub_pointcloud_;

        std::string input_topic_left_;
        std::string input_topic_right_;
        std::string output_topic_;
        std::string output_frame_id_;

        sensor_msgs::CameraInfo output_camera_info_left_;
        sensor_msgs::CameraInfo output_camera_info_right_;

        cv::Mat image_left_;
        cv::Mat undist_image_left_;
        cv::Mat image_right_;
        cv::Mat undist_image_right_;

        cv::Mat1f lmapx_;
        cv::Mat1f lmapy_;
        cv::Mat1f rmapx_;
        cv::Mat1f rmapy_;

        std::string param_file_path_;
        int SGBM_;
        int process_every_nth_frame_ = 1;
        int frame_counter_ = 0;
        double scale_ = 1.0;

        bool enable_dyn_reconf_;
        // stereo parameters
        int pre_filter_cap_;
        int sad_window_size_;
        int min_disparity_;
        int num_disparities_;
        int uniqueness_ratio_;
        int speckle_range_;
        int speckle_window_size_;

        // bm parameters
        int texture_threshold_;
        int pre_filter_type_;
        int pre_filter_size_;

        // sgbm parameters
        bool use_sgbm_;
        int sgbm_mode_;
        int p1_;
        int p2_;
        int disp_12_max_diff_;

        bool do_median_blur_;
        // output stereo
        float stereo_cx;
        float stereo_cy;
        float focal_length;
        float baseline = 0.064;

        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
            MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy>
            Sync;
    };
} // namespace t265_depth
