/*******************************************************************************************
 * File:        closeloop_node.cpp
 * 
 * Author:      Rui Chen (ruiche[at]umich.edu)
 * Last Modified:     10/01/2022
 * 
 * Description: Detect close loop
*******************************************************************************************/
#include <closeloop.h>

// void *wait(void *t) {
//     pthread_mutex_lock(&mutex_);
//     long tid = (long)t;
//     std::cout << "Sleeping in thread " << std::endl;
//     std::cout << "Thread with id : " << tid << "  ...exiting " << std::endl;
//     pthread_mutex_unlock(&mutex_);
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "closeloop");
    ros::NodeHandle nh;
    std::cout << "Initial vocabulary loaded!" << std::endl;
    // int i, rc;
    // pthread_t threads[NUM_THREADS];

    // for( i = 0; i < NUM_THREADS; i++ ) {
    //   std::cout << "main() : creating thread, " << i << std::endl;
    //   rc = pthread_create(&threads[i], NULL, wait, (void *)i); 
    //   if (rc) {
    //      std::cout << "Error:unable to create thread," << rc << std::endl;
    //      exit(-1);
    //   }
    // }
    
    // for( i = 0; i < NUM_THREADS; i++ ) {
    //     rc = pthread_join(threads[i], NULL);
    //     if (rc) {
    //         std::cout << "Error:unable to join," << rc << std::endl;
    //         exit(-1);
    //     }
    //     std::cout << "Main: completed thread id :" << i ;
    // }

    // std::cout << "Here, sleeping." << std::endl;
    // pthread_exit(NULL);
    // sleep(10);
    closeloop::CloseloopClass closeloop(nh);
    std::cout << "Closeloop Working " << std::endl;
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    return 0;
}
