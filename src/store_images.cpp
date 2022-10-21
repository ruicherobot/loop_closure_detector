/*******************************************************************************************
 * File:        store_images.cpp
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     10/01/2022
 * 
 * Description: Store images from ros topics for further better volcabulary training
*******************************************************************************************/

#include <store_images.h>

namespace closeloop{

    StoreImagesClass::StoreImagesClass(ros::NodeHandle& nh): nh_(nh), it_(nh){
        count_ = 0;
        index_ = 0;
        camera_sub_ = it_.subscribe("/camera/color/image_raw", 10, &StoreImagesClass::getCameraCallBack, this);
    }

    void StoreImagesClass::getCameraCallBack(const sensor_msgs::ImageConstPtr& msg)
    {
        count_++;
        try
        {
            cv_ptr_ = cv_bridge::toCvCopy(msg); //encoding: RGB8
            image_ = cv_ptr_->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        // cv::circle(image_, cv::Point(image_.cols/ 2, image_.rows/ 2), 10, cv::Scalar(255, 0, 255), 3);
        // cv::imshow("image", image_);
        if(count_ == 10){
            count_ = 0;
            index_++;
            cv::imwrite("/home/bipedlab/data/" + std::to_string(index_) + ".jpg", image_);
            std::cout<<"Store Images "<<index_<<std::endl;
        }
    }
}
