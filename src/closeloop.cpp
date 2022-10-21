/*******************************************************************************************
 * File:        closeloop.cpp
 *
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     10/01/2022
 *
 * Description: This version is a offline version
 *               First, we record edges and vertexes while we detect close loops
 *                 Second, as we detct loop closure, I will define the final edge myself for now, after many edge cases are eliminated, this will be automatic
*******************************************************************************************/

#include <closeloop.h>

namespace closeloop{

    CloseloopClass::CloseloopClass(ros::NodeHandle& nh): nh_(nh), it_(nh){

        nh_.getParam("closeloop/vocab_initial_path", vocab_initial_path_);
        nh_.getParam("closeloop/vocab_smaller_path", vocab_smaller_path_);
        nh_.getParam("closeloop/vocab_larger_path", vocab_larger_path_);
        nh_.getParam("closeloop/txt_vertexes_path", txt_vertexes_path_);
        nh_.getParam("closeloop/txt_edges_path", txt_edges_path_);
        nh_.getParam("closeloop/g2o_path", g2o_path_);
        nh_.getParam("closeloop/pcd_path", pcd_path_);
        nh_.getParam("closeloop/max_range", max_range_);
        nh_.getParam("closeloop/height_threshold", height_threshold_);
        nh_.getParam("closeloop/view_direction", view_direction_);
        nh_.getParam("closeloop/view_angle", view_angle_);

        vocab_.load(vocab_initial_path_);
        if (vocab_.empty()) {
            std::cout << "XXXXXXXXXX Vocabulary cannot be opened. XXXXXXXXXXX" << std::endl;
        }
        DBoW3::Database db(vocab_, false, 0);
        db_ = db;
        db.clear();
        count_odo_ = 0;
        count_g2o_ = 0;
        pose_test_visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("test_visualization", 1);
        state_sub_ = nh_.subscribe("/cassie/inekf_state", 10, &CloseloopClass::updateState, this);
        camera_sub_ = it_.subscribe("/camera/color/image_raw", 10, &CloseloopClass::getCameraCallBack, this);
        pointcloud_sub_ = nh_.subscribe("/velodyne_points", 10, &CloseloopClass::getVelodyneCallBack, this);
        // This is better so far

        // vocab_.load(vocab_smaller_path);
        // //DBoW3::Vocabulary vocab(vocab_larger_path_);
        // std::cout << "HERE: Load New Vocab!" << std::endl;
        // DBoW3::Database db(vocab_, false, 0);
        // db_ = db;
        // db.clear();
        // std::cout << "HERE: DB Created!" << std::endl;

    }

    void CloseloopClass::updateState(inekf_msgs::State input)
    {
        count_odo_++;

        if(count_odo_ == 1){
            std::ofstream vertex_se3_quat;
            vertex_se3_quat.open(txt_vertexes_path_, std::ofstream::app);
            vertex_se3_quat.precision(8);

            odom_pre_ = input.position;
            quaternion_pre_ = input.orientation;

            if (vertex_se3_quat.is_open())
            {
                // VERTEX_SE3:QUAT i x y z qx qy qz qw
                vertex_se3_quat << std::fixed << "VERTEX_SE3:QUAT " << count_g2o_
                                                            << " " << input.position.x
                                                            << " " << input.position.y
                                                            << " " << input.position.z
                                                            << " " << input.orientation.x
                                                            << " " << input.orientation.y
                                                            << " " << input.orientation.z
                                                            << " " << input.orientation.w << std::endl;
                vertex_se3_quat.close();

                //save pcl file
                if (cloud_.size() > 0)
                {
                    pcl::PointCloud<pcl::PointXYZI> trans_cloud;

                    pcl_lock_.lock();
                    cloud_.header = pcl_conversions::toPCL(input.header);
                    cloud_.header.frame_id = std::string("odom");
                    tf_listener_.waitForTransform(std::string("velodyne"), std::string("odom"),
                                        input.header.stamp, ros::Duration(0.1));
                    bool is_successful = pcl_ros::transformPointCloud(
                            std::string("velodyne"), cloud_, trans_cloud, tf_listener_);
                    cloud_.clear();
                    pcl_lock_.unlock();

                    if (is_successful)
                    {
                        pcl::io::savePCDFileASCII (pcd_path_ +
                                                   std::string("pointcloud") +
                                                   std::to_string(count_g2o_) +
                                                   std::string(".pcd"),
                                                   trans_cloud);
                    }
                }

            }
        }

        if(count_odo_ == 10000){
            count_odo_ = 2;
            count_g2o_++;

            std::ofstream edge_se3_quat;
            edge_se3_quat.open(txt_edges_path_, std::ofstream::app);
            edge_se3_quat.precision(6);
            if (edge_se3_quat.is_open())
            {
                tf::Quaternion quaternion_tf_pre, quaternion_tf;
                tf::quaternionMsgToTF(quaternion_pre_, quaternion_tf_pre);
                tf::Matrix3x3(quaternion_tf).getRPY(pre_roll_, pre_pitch_, pre_yaw_);
                tf::quaternionMsgToTF(input.orientation, quaternion_tf);
                tf::Matrix3x3(quaternion_tf).getRPY(roll_, pitch_, yaw_);
                geometry_msgs::Quaternion quaternion_temp = tf::createQuaternionMsgFromRollPitchYaw(roll_ - pre_roll_, pitch_ - pre_pitch_, yaw_ - pre_yaw_);
                // EDGE_SE3:QUAT i j
                // x y z
                // qx qy qz qw
                // [q11] q12 q13 q14 q15 q16 [q22] q23 q24 q25 q26 [q33] q34 q35 q36 [q44] q45 q46 [q55] q56 [q66]

                // edge_se3_quat << std::fixed << "EDGE_SE3:QUAT " << count_g2o_ - 1
                //                                          << " " << count_g2o_
                //                                          << " " << input.position.x - odom_pre_.x
                //                                          << " " << input.position.y - odom_pre_.y
                //                                          << " " << input.position.z - odom_pre_.z
                //                                          << " " << quaternion_temp.x
                //                                          << " " << quaternion_temp.y
                //                                          << " " << quaternion_temp.z
                //                                          << " " << quaternion_temp.w
                //                                          << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0
                //                                          << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0
                //                                          << " " << 1.0 << " " << 0.0 << " " << 0.0 << " " << 0.0
                //                                          << " " << 1.0 << " " << 0.0 << " " << 0.0
                //                                          << " " << 1.0 << " " << 0.0
                //                                          << " " << 1.0 << std::endl;

                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                for( int idx = 0; idx < 81; idx++){
                    cov_for_all << input.covariance[idx];
                }
                inv_cov_for_all = cov_for_all.inverse();
                Matrix6d inv_cov_temp = inv_cov_for_all.block<6 ,6>(3, 3);
                edge_se3_quat << std::fixed << "EDGE_SE3:QUAT " << count_g2o_ - 1
                                                         << " " << count_g2o_
                                                         << " " << input.position.x - odom_pre_.x
                                                         << " " << input.position.y - odom_pre_.y
                                                         << " " << input.position.z - odom_pre_.z
                                                         << " " << quaternion_temp.x
                                                         << " " << quaternion_temp.y
                                                         << " " << quaternion_temp.z
                                                         << " " << quaternion_temp.w
                                                         << " " << inv_cov_for_all(3, 3) << " " << inv_cov_for_all(3, 4) << " " << inv_cov_for_all(3, 5) << " " << inv_cov_for_all(3, 6) << " " << inv_cov_for_all(3, 7) << " " << inv_cov_for_all(3, 8)
                                                         << " " << inv_cov_for_all(4, 4) << " " << inv_cov_for_all(4, 5) << " " << inv_cov_for_all(4, 6) << " " << inv_cov_for_all(4, 7) << " " << inv_cov_for_all(4, 8)
                                                         << " " << inv_cov_for_all(5, 5) << " " << inv_cov_for_all(5, 6) << " " << inv_cov_for_all(5, 7) << " " << inv_cov_for_all(5, 8)
                                                         << " " << inv_cov_for_all(6, 6) << " " << inv_cov_for_all(6, 7) << " " << inv_cov_for_all(6, 8)
                                                         << " " << inv_cov_for_all(7, 7) << " " << inv_cov_for_all(7, 8)
                                                         << " " << inv_cov_for_all(8, 8) << std::endl;
                edge_se3_quat.close();
                ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            }

            std::ofstream vertex_se3_quat;
            vertex_se3_quat.open(txt_vertexes_path_, std::ofstream::app);
            vertex_se3_quat.precision(6);
            if (vertex_se3_quat.is_open())
            {
                // VERTEX_SE3:QUAT i x y z qx qy qz qw
                vertex_se3_quat << std::fixed << "VERTEX_SE3:QUAT " << count_g2o_
                                                         << " " << input.position.x
                                                         << " " << input.position.y
                                                         << " " << input.position.z
                                                         << " " << input.orientation.x
                                                         << " " << input.orientation.y
                                                         << " " << input.orientation.z
                                                         << " " << input.orientation.w << std::endl;
                vertex_se3_quat.close();

                //save pcl file
                if (cloud_.size() > 0)
                {
                    pcl::PointCloud<pcl::PointXYZI> trans_cloud;

                    pcl_lock_.lock();
                    cloud_.header = pcl_conversions::toPCL(input.header);
                    cloud_.header.frame_id = std::string("odom");
                    tf_listener_.waitForTransform(std::string("velodyne"), std::string("odom"),
                                        input.header.stamp, ros::Duration(0.1));
                    bool is_successful = pcl_ros::transformPointCloud(
                            std::string("velodyne"), cloud_, trans_cloud, tf_listener_);
                    cloud_.clear();
                    pcl_lock_.unlock();

                    if (is_successful)
                    {
                        pcl::io::savePCDFileASCII (pcd_path_ +
                                                   std::string("pointcloud") +
                                                   std::to_string(count_g2o_) +
                                                   std::string(".pcd"),
                                                   trans_cloud);
                    }
                }
            }

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "test_visualization";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = input.position.x;
            marker.pose.position.y = input.position.y;
            marker.pose.position.z = 0;
            marker.pose.orientation = input.orientation;
            marker.scale.x = 4;
            marker.scale.y = 2;
            marker.scale.z = 1;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0;
            pose_test_visualization_pub_.publish(marker);
            std::cout<<"PUBLISHED!"<<std::endl;

            odom_pre_ = input.position;
            quaternion_pre_ = input.orientation;

        }

    }

    void CloseloopClass::getCameraCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg); //encoding: RGB8
            image_ = cv_ptr_->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // cv::imshow("image", image_);
        if(count_image_ == 20){
            count_image_ = 0;
            images_.push_back(image_);

            cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptor;
            detector->detectAndCompute(image_, cv::Mat(), keypoints, descriptor);
            descriptors_.push_back(descriptor);
            db_.add(descriptor);
            // we can compare the images directly or we can compare one image to a database image

            DBoW3::BowVector vq;
            vocab_.transform(descriptor, vq);

            std::vector<double> scores;
            for (auto &dtemp : descriptors_) {
                DBoW3::BowVector vtemp;
                vocab_.transform(dtemp, vtemp);
                scores.push_back(vocab_.score(vq, vtemp));
            }

            DBoW3::QueryResults ret;
            db_.query(descriptor, ret, 4);
            std::cout << ret << std::endl;

        }
        count_image_++;
    }

    // Approximation for J_R^{-1}
    Matrix6d CloseloopClass::getJRInv(const Sophus::SE3d &e) {
        Matrix6d J;
        J.block(0, 0, 3, 3) = Sophus::SO3d::hat(e.so3().log());
        J.block(0, 3, 3, 3) = Sophus::SO3d::hat(e.translation());
        J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero(3, 3);
        J.block(3, 3, 3, 3) = Sophus::SO3d::hat(e.so3().log());
        // J = J * 0.5 + Matrix6d::Identity();
        J = Matrix6d::Identity();
        return J;
    }

    void CloseloopClass::getVelodyneCallBack(const sensor_msgs::PointCloud2 &msg)
    {
        pcl::PointCloud<pcl::PointXYZI> received_pointcloud;
        pcl::fromROSMsg(msg, received_pointcloud);

        //transform pointcloud
        pcl::PointCloud<pcl::PointXYZI> new_cloud;
        for (std::size_t i = 0; i < received_pointcloud.size(); ++i)
        {
            double x = received_pointcloud[i].x;
            double y = received_pointcloud[i].y;
            double z = received_pointcloud[i].z;
            double angle = std::atan2(std::abs(y), x) * 180 / M_PI;
            if (y < 0)
            {
                angle = -angle;
            }
            double angle_diff = std::abs(angle - view_direction_);
            if (std::sqrt(x*x + y*y) <= max_range_ && z <= height_threshold_
                && angle_diff <= view_angle_ / 2)
            {
                new_cloud.push_back(received_pointcloud[i]);
            }
        }

        //publish transformed pointcloud
        sensor_msgs::PointCloud2 new_msg;
        pcl::toROSMsg(new_cloud, new_msg);
        new_msg.header = msg.header;

        tf_listener_.waitForTransform(std::string("odom"), std::string("velodyne"),
                            msg.header.stamp, ros::Duration(1.0));
        sensor_msgs::PointCloud2 msg_world;
        bool is_successful = pcl_ros::transformPointCloud(
                std::string("odom"), new_msg, msg_world, tf_listener_);
        if (!is_successful)
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI> raw_pointcloud;
        pcl::fromROSMsg(msg_world, raw_pointcloud);

        pcl_lock_.lock();
        cloud_ += raw_pointcloud;
        pcl_lock_.unlock();
        return;
    }

}
