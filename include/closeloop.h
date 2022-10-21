/*******************************************************************************************
 * File:        closeloop.h
 *
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     07/04/2022
 *
 * Description: closeloop
*******************************************************************************************/
#ifndef CLOSELOOP_H
#define CLOSELOOP_H
// #pragma once
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <pthread.h>
#include <unistd.h>
//ros and tf
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Eigen>

// DBoW and openCV
#include "DBoW3/DBoW3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include "inekf_msgs/State.h"
#include "inekf_msgs/LocalizationState.h"

// #include <gtsam/geometry/Pose2.h>
// #include <gtsam/nonlinear/GaussNewtonOptimizer.h>
// #include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
// #include <gtsam/slam/BetweenFactor.h>
// #include <gtsam/nonlinear/Values.h>
// #include <gtsam/nonlinear/Marginals.h>
// #include <gtsam/nonlinear/utilities.h>
// #include <gtsam/nonlinear/ISAM2.h>

#include <sophus/se3.hpp>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/impl/point_types.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "boost/thread/mutex.hpp"
#include "boost/thread.hpp"

#define NUM_THREADS 15

namespace closeloop {

    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    typedef Eigen::Matrix<double, 6, 6> Matrix6d;

    typedef Eigen::Matrix<double, 9, 9> Matrix9d;

    class CloseloopClass {

    public:
        CloseloopClass(ros::NodeHandle& nh);

        pthread_mutex_t mutex_;

        std::string vocab_initial_path_;

        std::string vocab_smaller_path_;

        std::string vocab_larger_path_;

        std::string txt_vertexes_path_;

        std::string txt_edges_path_;

        std::string g2o_path_;

        std::string pcd_path_;

        ros::NodeHandle nh_;

        ros::Subscriber state_sub_;

        ros::Subscriber pointcloud_sub_;

        ros::Publisher pose_test_visualization_pub_;

        image_transport::ImageTransport it_;

        image_transport::Subscriber camera_sub_;

        cv_bridge::CvImagePtr cv_ptr_;

        int count_odo_;

        int count_g2o_;

        int count_image_;

        int img_g2o_;

        double roll_;

        double pitch_;

        double yaw_;

        double pre_roll_;

        double pre_pitch_;

        double pre_yaw_;

        Matrix9d cov_for_all;

        Matrix9d inv_cov_for_all;

        double max_range_;

        double height_threshold_;

        double view_direction_;

        double view_angle_;

        geometry_msgs::Point odom_pre_;

        geometry_msgs::Quaternion quaternion_pre_;

        cv::Mat image_;

        std::vector<cv::Mat> images_;

        DBoW3::Vocabulary vocab_;

        DBoW3::Database db_;

        std::vector<cv::Mat> descriptors_;

        pcl::PointCloud<pcl::PointXYZI> cloud_;

        boost::mutex pcl_lock_;

        tf::TransformListener tf_listener_;

        void updateState(inekf_msgs::State input);

        void getCameraCallBack(const sensor_msgs::ImageConstPtr& msg);

        void getVelodyneCallBack(const sensor_msgs::PointCloud2 &msg);

        Matrix6d getJRInv(const Sophus::SE3d &e);

    };
}

#endif /* CLOSELOOP_H */
