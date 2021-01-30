#include <t265_depth/t265_depth.h>

namespace t265_depth
{
    t265Depth::t265Depth(ros::NodeHandle &node, ros::NodeHandle &private_node)
        : it_(node)
    {
        private_node.getParam("scale", scale_);
        private_node.getParam("process_every_nth_frame", process_every_nth_frame_);
        private_node.getParam("input_topic_left", input_topic_left_);
        private_node.getParam("input_topic_right", input_topic_right_);
        private_node.getParam("output_frame_id", output_frame_id_);
        private_node.getParam("param_file_path", param_file_path_);
        ROS_INFO_STREAM_THROTTLE(2, "Reading Intrinsics from: " << param_file_path_);
        ROS_INFO("Scale set to: %f", scale_);

        std::string pre_filter_type_string;
        private_node.param("pre_filter_type", pre_filter_type_string, kPreFilterType);

        if (pre_filter_type_string == std::string("xsobel"))
        {
            pre_filter_type_ = cv::StereoBM::PREFILTER_XSOBEL;
        }
        else if (pre_filter_type_string == std::string("normalized_response"))
        {
            pre_filter_type_ = cv::StereoBM::PREFILTER_NORMALIZED_RESPONSE;
        }
        else
        {
            throw std::runtime_error(
                "Unrecognized prefilter type, choices are 'xsobel' or "
                "'normalized_response'");
        }
        // general stereo parameters
        private_node.param("pre_filter_cap", pre_filter_cap_, kPreFilterCap);
        private_node.param("sad_window_size", sad_window_size_, kSADWindowSize);
        private_node.param("min_disparity", min_disparity_, kMinDisparity);
        private_node.param("num_disparities", num_disparities_, kNumDisparities);
        private_node.param("uniqueness_ratio", uniqueness_ratio_, kUniquenessRatio);
        private_node.param("speckle_range", speckle_range_, kSpeckleRange);
        private_node.param("speckle_window_size", speckle_window_size_, kSpeckleWindowSize);
        private_node.param("sad_window_size", sad_window_size_, kSADWindowSize);
        // bm parameters
        private_node.param("texture_threshold", texture_threshold_, kTextureThreshold);
        private_node.param("pre_filter_size", pre_filter_size_, kPreFilterSize);
        // sgbm parameters
        private_node.param("use_sgbm", use_sgbm_, kUseSGBM);
        private_node.param("sgbm_mode", sgbm_mode_, kSGBMMode);
        private_node.param("p1", p1_, kP1);
        private_node.param("p2", p2_, kP2);
        private_node.param("disp_12_max_diff", disp_12_max_diff_, kDisp12MaxDiff);

        private_node.param("do_median_blur", do_median_blur_, kDoMedianBlur);

        private_node.param("enable_dyn_reconf", enable_dyn_reconf_, kEnableDynReconf);

        initializeRectificationMapping(param_file_path_);
        // Publishers
        pub_img_left_rect_ = it_.advertise(input_topic_left_ + "/rectified", 10);
        pub_img_right_rect_ = it_.advertise(input_topic_right_ + "/rectified", 10);
        pub_camera_info_left_ = node.advertise<sensor_msgs::CameraInfo>(input_topic_left_ + "/rectified" + "/info", 10);
        pub_camera_info_right_ = node.advertise<sensor_msgs::CameraInfo>(input_topic_right_ + "/rectified" + "/info", 10);
        pub_disparity_ = it_.advertise("/disparity", 10);
        pub_pointcloud_ = node.advertise<sensor_msgs::PointCloud2>("/points2", 10);
        // Subscriber - sync policy
        message_filters::Subscriber<sensor_msgs::Image> *image_sub_L_ptr;
        message_filters::Subscriber<sensor_msgs::Image> *image_sub_R_ptr;
        Sync *sync_ptr;
        image_sub_L_ptr = new message_filters::Subscriber<sensor_msgs::Image>();
        image_sub_R_ptr = new message_filters::Subscriber<sensor_msgs::Image>();
        image_sub_L_ptr->subscribe(private_node, input_topic_left_, 1);
        image_sub_R_ptr->subscribe(private_node, input_topic_right_, 1);

        sync_ptr = new Sync(MySyncPolicy(10), *image_sub_L_ptr, *image_sub_R_ptr);
        sync_ptr->registerCallback(boost::bind(&t265Depth::syncCallback, this, _1, _2));
    }

    t265Depth::~t265Depth()
    {
    }

    void t265Depth::syncCallback(const sensor_msgs::Image::ConstPtr &image_msg_left,
                                 const sensor_msgs::Image::ConstPtr &image_msg_right)
    {
        // ROS_INFO_STREAM_THROTTLE(2, "In syncCallback");

        if (++frame_counter_ < process_every_nth_frame_)
        {
            return;
        }
        frame_counter_ = 0;

        sensor_msgs::ImagePtr out_img_msg_left;
        sensor_msgs::ImagePtr out_img_msg_right;
        image_left_ = cv_bridge::toCvCopy(image_msg_left, "mono8")->image;
        image_right_ = cv_bridge::toCvCopy(image_msg_right, "mono8")->image;
        std_msgs::Header header_out = image_msg_left->header;
        if (scale_ < 1)
        {
            resize(image_left_, image_left_, Size(image_left_.cols * scale_, image_left_.rows * scale_));
            resize(image_right_, image_right_, Size(image_right_.cols * scale_, image_right_.rows * scale_));
        }
        elaborateImages(header_out);
        try
        {
            out_img_msg_left = cv_bridge::CvImage(std_msgs::Header(), "mono8", undist_image_left_).toImageMsg();
            out_img_msg_right = cv_bridge::CvImage(std_msgs::Header(), "mono8", undist_image_right_).toImageMsg();
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'mono8'.", image_msg_left->encoding.c_str());
            return;
        }
        // left
        out_img_msg_left->header = image_msg_left->header;
        out_img_msg_left->header.frame_id = output_frame_id_;
        output_camera_info_left_.header = image_msg_left->header;
        output_camera_info_left_.header.frame_id = output_frame_id_;
        //pub left
        if (pub_img_left_rect_.getNumSubscribers() > 0)
        {
        pub_img_left_rect_.publish(out_img_msg_left);
        }
        if (pub_camera_info_left_.getNumSubscribers() > 0)
        {
        pub_camera_info_left_.publish(output_camera_info_left_);
        }
        // right
        out_img_msg_right->header = image_msg_right->header;
        out_img_msg_right->header.frame_id = output_frame_id_;
        output_camera_info_left_.header = image_msg_right->header;
        output_camera_info_left_.header.frame_id = output_frame_id_;
        //pub right
        if (pub_img_right_rect_.getNumSubscribers() > 0)
        {
        pub_img_right_rect_.publish(out_img_msg_right);
        }
        if (pub_camera_info_right_.getNumSubscribers() > 0)
        {
        pub_camera_info_right_.publish(output_camera_info_right_);
        }
    }

    void t265Depth::publishCameraInfo(cv::Mat K1, cv::Mat K2, cv::Mat P1, cv::Mat P2, cv::Mat R1, cv::Mat R2)
    {
        // Copy the parameters for rectified images to the camera_info messages
        for (int i = 0; i < 9; i++)
        {
            output_camera_info_left_.K[i] = K1.at<float>(i);
            output_camera_info_right_.K[i] = K2.at<float>(i);
            output_camera_info_left_.R[i] = R1.at<float>(i);
            output_camera_info_right_.R[i] = R2.at<float>(i);
        }
        for (int i = 0; i < 12; i++)
        {
            output_camera_info_left_.P[i] = P1.at<float>(i);
            output_camera_info_right_.P[i] = P2.at<float>(i);
        }
    }

    void t265Depth::initializeRectificationMapping(std::string param_file_path)
    {
        cv::Mat1f Q, P1, P2;
        cv::Mat1f R1, R2, K1, K2, D1, D2, R;
        cv::Vec3f T;
        cv::Vec2f size_input, size_output;

        cv::FileStorage param_file = cv::FileStorage(param_file_path, cv::FileStorage::READ);

        param_file["K1"] >> K1;
        param_file["K2"] >> K2;
        param_file["D1"] >> D1;
        param_file["D2"] >> D2;
        param_file["R"] >> R;
        param_file["T"] >> T;

        param_file["input"] >> size_input;
        param_file["output"] >> size_output;

        // The resolution of the input images used for stereo calibration.
        cv::Size input_img_size(size_input[0], size_input[1]);
        // The resolution of the output rectified images. Lower resolution images require less computation time.
        cv::Size output_img_size(size_output[0], size_output[1]);

        output_camera_info_left_.width = size_output[0];
        output_camera_info_left_.height = size_output[1];
        output_camera_info_left_.D = std::vector<double>(5, 0);

        output_camera_info_right_.width = size_output[0];
        output_camera_info_right_.height = size_output[1];
        output_camera_info_right_.D = std::vector<double>(5, 0);

        // Rectification
        int max_disp = min_disparity_ + num_disparities_;
        float stereo_fov_rad = 90 * (M_PI / 180); // 90 degree desired fov
        // float stereo_height_px = 300;    // 300x300 pixel stereo output
        float stereo_width_px = size_output[0] + max_disp;
        float stereo_height_px = size_output[1];
        float stereo_focal_px = stereo_height_px / 2 / tan(stereo_fov_rad / 2);

        cv::Mat R_left = cv::Mat::eye(3, 3, CV_32F);
        cv::Mat R_right = R;

        cv::Size stereo_size = cv::Size(stereo_width_px, stereo_height_px);

        stereo_cx = (stereo_width_px - 1) / 2 + max_disp;
        stereo_cy = (stereo_height_px - 1) / 2;

        cout << "stereo_focal_px: " << stereo_focal_px << "\n"
             << "stereo_height_px: " << stereo_height_px << "\n"
             << "stereo_width_px: " << stereo_width_px << "\n"
             << "stereo_cx: " << stereo_cx << "\n"
             << "stereo_cy: " << stereo_cy << "\n"
             << "max_disp: " << max_disp << "\n";

        // P_left  //Set mid point to actual pixel center
        cv::Mat_<float> P_left(3, 4);
        P_left << stereo_focal_px, 0, stereo_cx, 0,
            0, stereo_focal_px, stereo_cy, 0,
            0, 0, 1, 0;
        // P_right gets the baseline added
        cv::Mat_<float> P_right(3, 4);
        P_left.copyTo(P_right);
        P_right.at<float>(0, 3) = T[0] * stereo_focal_px;

        Q = (cv::Mat_<float>(4, 4) << 1, 0, 0, -(stereo_cx - max_disp),
             0, 1, 0, -stereo_cy,
             0, 0, 0, stereo_focal_px,
             0, 0, (-1 / T[0]), 0);

        cv::fisheye::initUndistortRectifyMap(K1, D1, R_left, P_left, stereo_size, CV_32FC1, lmapx_, lmapy_);
        cv::fisheye::initUndistortRectifyMap(K2, D2, R_right, P_right, stereo_size, CV_32FC1, rmapx_, rmapy_);
        publishCameraInfo(K1, K2, P_left, P_right, R_left, R_right);
        focal_length = P_left.at<float>(0, 0);
        baseline = std::abs(T[0]);
        cout << "focal_length: " << focal_length << "\n"
             << "baseline: " << baseline << "\n";
        ROS_INFO("[t265Depth] Initialization complete");
    }

    void t265Depth::computePointcloud(const cv::Mat &input_disparity,
                                      sensor_msgs::PointCloud2 &pointcloud)
    {
        // build pointcloud
        int side_bound = sad_window_size_ / 2;
        sensor_msgs::PointCloud2Modifier modifier(pointcloud);
        pointcloud.width = input_disparity.cols - side_bound;
        pointcloud.height = input_disparity.rows - side_bound;
        modifier.setPointCloud2FieldsByString(1, "xyz");

        sensor_msgs::PointCloud2Iterator<float> pointcloud_x(pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> pointcloud_y(pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> pointcloud_z(pointcloud, "z");

        for (int y_pixels = side_bound; y_pixels < input_disparity.rows - side_bound;
             ++y_pixels)
        {
            for (int x_pixels = side_bound + min_disparity_ + num_disparities_;
                 x_pixels < input_disparity.cols - side_bound; ++x_pixels)
            {
                const int16_t &input_value = input_disparity.at<int16_t>(y_pixels, x_pixels);
                double disparity_value;
                disparity_value = static_cast<double>(input_value);

                // double min, max;
                // cv::minMaxLoc(input_disparity, &min, &max);
                // std::cout << "min: " << min << " max: " << max << std::endl;
                // min: -16 max: 1008

                // the 16* is needed as opencv stores disparity maps as 16 * the true
                // values
                if (disparity_value > 50 && disparity_value < 800)
                {
                    *pointcloud_z = (16 * focal_length * baseline) / disparity_value;
                    *pointcloud_x = *pointcloud_z * (x_pixels - stereo_cx) / focal_length;
                    *pointcloud_y = *pointcloud_z * (y_pixels - stereo_cy) / focal_length;
                }

                ++pointcloud_x;
                ++pointcloud_y;
                ++pointcloud_z;
            }
        }
    }

    void t265Depth::elaborateImages(const std_msgs::Header &header_msg)
    {
        // start depth image operations
        cv::Mat left_disp, left_disp8u, right_disp, right_disp8u;
        cv::Mat filtered_disp, filtered_disp8u;

        if (image_left_.rows > 0 && image_right_.rows > 0)
        {
            cv::remap(image_left_, undist_image_left_, lmapx_, lmapy_, cv::INTER_LINEAR);
            cv::remap(image_right_, undist_image_right_, rmapx_, rmapy_, cv::INTER_LINEAR);

            if (use_sgbm_)
            {
                int mode;
                if (sgbm_mode_ == 1)
                {
                    mode = cv::StereoSGBM::MODE_HH;
                }
                else if (sgbm_mode_ == 2)
                {
                    mode = cv::StereoSGBM::MODE_SGBM_3WAY;
                }
                else
                {
                    mode = cv::StereoSGBM::MODE_SGBM;
                }

                cv::Ptr<cv::StereoSGBM> left_matcher = cv::StereoSGBM::create(
                    min_disparity_,
                    num_disparities_,
                    sad_window_size_,
                    p1_,
                    p2_,
                    disp_12_max_diff_,
                    pre_filter_cap_,
                    uniqueness_ratio_,
                    speckle_window_size_,
                    speckle_range_,
                    mode);
                left_matcher->compute(undist_image_left_, undist_image_right_, left_disp);
            }
            else
            {
                cv::Ptr<cv::StereoBM> left_matcher =
                    cv::StereoBM::create(num_disparities_, sad_window_size_);

                left_matcher->setPreFilterType(pre_filter_type_);
                left_matcher->setPreFilterSize(pre_filter_size_);
                left_matcher->setPreFilterCap(pre_filter_cap_);
                left_matcher->setMinDisparity(min_disparity_);
                left_matcher->setTextureThreshold(texture_threshold_);
                left_matcher->setUniquenessRatio(uniqueness_ratio_);
                left_matcher->setSpeckleRange(speckle_range_);
                left_matcher->setSpeckleWindowSize(speckle_window_size_);
                left_matcher->compute(undist_image_left_, undist_image_right_, left_disp);
            }
            if (do_median_blur_)
            {
                cv::medianBlur(left_disp, left_disp, 3);
            }

            if (pub_disparity_.getNumSubscribers() > 0)
            {
            cv::normalize(left_disp, left_disp8u, 0, 255, cv::NORM_MINMAX, CV_8U);
            // publish disparity image
            sensor_msgs::ImagePtr out_disparity_msg;
            out_disparity_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", left_disp8u).toImageMsg();
            pub_disparity_.publish(out_disparity_msg);
            }
            // build pointcloud
            sensor_msgs::PointCloud2 pointcloud_msg;
            computePointcloud(left_disp, pointcloud_msg);
            // publish pointcloud
            pointcloud_msg.header.stamp = header_msg.stamp;
            pointcloud_msg.header.frame_id = output_frame_id_;

            if (pub_pointcloud_.getNumSubscribers() > 0)
            {
            pub_pointcloud_.publish(pointcloud_msg);
            }
        }
    }

} // namespace t265_depth
