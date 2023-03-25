#include <librealsense2/rs.hpp>


#include "realsense_health_checker.hpp"  

#include "mains_examples.hpp"









/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


int main(int argc, char * argv[]){

  rclcpp::init(argc, argv);
  std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";

  auto realsenseNode = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>(
    std::string("realsense_error_node"),
    path,
    std::string("realsense_error_topic")
  );
  
  

 //realsenseEerorschecker::RealSenseErrorsChecker realsenseErrors();
// rclcpp::spin(realsenseNode);
 
 try
  {   
    
    //publisher->publish("OK");
    //publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    realsenseNode->start_msg_boot_up_msg();
    test_object_detection_main();
    }
    // Here is about to catch the potential error that could occur!
    catch (const rs2::camera_disconnected_error& e)
    {
    realsenseNode->error_catcher_writer_publisher(e);
    printf("DICO 1\n");
    
    }
    catch (const rs2::error& e)
    {
      
        if (e.get_type() == RS2_EXCEPTION_TYPE_CAMERA_DISCONNECTED) {
    // Handle camera disconnection error
        realsenseNode->error_catcher_writer_publisher(e);
        printf("DICO 2\n");
    
        }
    }
    catch (const std::exception& e)
    {
    realsenseNode->error_catcher_writer_publisher(e);
        printf("DICO 3\n");

    
  }
  
  rclcpp::spin(realsenseNode);
  rclcpp::shutdown();

};
