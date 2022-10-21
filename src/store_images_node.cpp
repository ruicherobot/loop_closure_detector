/*******************************************************************************************
 * File:        store_images_node.cpp
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     10/01/2022
 * 
 * Description: Store images from ros topics for further better volcabulary training
*******************************************************************************************/
#include <store_images.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "store_images");
    ros::NodeHandle nh;
    closeloop::StoreImagesClass storeImages(nh);
    std::cout << "Storing Images Working " << std::endl;
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}
