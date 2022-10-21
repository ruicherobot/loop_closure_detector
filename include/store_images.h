/*******************************************************************************************
 * File:        store_images.h
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     07/02/2022
 * 
 * Description: closeloop
*******************************************************************************************/
#ifndef STORE_IMAGES_H
#define STORE_IMAGES_H
// #pragma once 
#include <iostream>
#include <cmath>

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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

//messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>


namespace closeloop {
    
    class StoreImagesClass {

    public:

        StoreImagesClass(ros::NodeHandle& nh);

        ros::NodeHandle nh_;
        
        image_transport::ImageTransport it_;

        image_transport::Subscriber camera_sub_;

        cv_bridge::CvImagePtr cv_ptr_;
        
        cv::Mat image_;

        int count_;

        int index_;

        void getCameraCallBack(const sensor_msgs::ImageConstPtr& msg);
    };
}

#endif /* STORE_IMAGES_H */