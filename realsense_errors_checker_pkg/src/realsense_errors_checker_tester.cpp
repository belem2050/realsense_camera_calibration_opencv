#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include <librealsense2/rs.hpp>

#include "realsense_health_checker.hpp"


// Mock rs2::error class




TEST(RealSenseErrorsChecker, TestErrorCatcherWriterPublisher) {
    rs2::context ctx;
    rs2::device dev;

    std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";

    auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");
    

  //dev = ctx.query_devices()[0];
  auto devices = ctx.query_devices();
  if (devices.size() > 0) {
    rs2::device dev = devices[0];
    rs2::pipeline pipe(ctx);
    pipe.start();
    printf("Nice 1 \n");
    rs2::sensor sensor = dev.query_sensors()[0];
  
    
    try {
        sensor.close();
    }
    catch (rs2::wrong_api_call_sequence_error& e) {
        EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
    }
    
    pipe.stop();
  }
  else {
    printf("No RealSense devices detected.");
  }
}



//
TEST(RealSenseErrorsChecker, TestErrorCatcherWriterPublisherCameraDisconnected) {
    rs2::context ctx;
    rs2::device dev;

    std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";

    auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");


  ASSERT_NO_THROW( dev = ctx.query_devices()[0]);
  // Disconnect the camera
  dev.hardware_reset();
  // Try to close the sensor before stopping the pipeline
    try {
      //dev.hardware_reset();
      dev.query_sensors()[0].get_stream_profiles();
      printf("DISCO 1\n");
    }
    catch (rs2::camera_disconnected_error& e) {
      // Verify that the error is caught and handled correctly
      printf("DISCO 2\n");
      EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
      
    }
}





TEST(RealSenseErrorsChecker, TestInvalidValueError) {
  rs2::context ctx;
  rs2::device dev;
   std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";

    auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");

  ASSERT_NO_THROW(dev = ctx.query_devices()[0]);

  try {
    // Set an invalid option on the depth sensor
    dev.first<rs2::depth_sensor>().set_option(rs2_option::RS2_OPTION_EXPOSURE, -50);
  } catch (rs2::backend_error& e) {
    // Verify that the error is caught and handled correctly
    EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
    
  }
}



TEST(RealSenseErrorsChecker, TestNotImplementedError) {
  rs2::context ctx;
  rs2::device dev;

  std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";
  auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");

 dev = ctx.query_devices()[0];

  try {
    // Try to enable an unsupported option on the color sensor
     rs2::sensor sensor = dev.query_sensors()[0];
    sensor.set_option(RS2_OPTION_AVALANCHE_PHOTO_DIODE,1);
    
  } catch (rs2::error& e) {
    // Verify that the error is caught and handled correctly
    EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
    
  }
  
  
}


TEST(RealSenseErrorsChecker, TestErrorCatcherWriterPublisherDeviceInRecoveryMode) {
  // Create a context and try to connect to a device
    rs2::context ctx;
    rs2::device dev;

    std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";
    auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");

    ASSERT_NO_THROW(dev = ctx.query_devices()[0]);

  // Put the device in recovery mode
  try {
    dev.hardware_reset();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  catch (rs2::error& e) {
    // Verify that the error is caught and handled correctly
    EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
    
  }
  
  
}


TEST(RealSenseErrorsChecker, TestErrorCatcherWriterPublisherUnrecoverable) {
    rs2::context ctx;
    rs2::device dev;
    std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";
    auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");

    
    dev = ctx.query_devices()[0];
    rs2::sensor sensor = dev.query_sensors()[0];

  // Try to set an invalid option
  try {
    sensor.set_option(rs2_option(9999), 42);
    
  }
  catch (rs2::error& e) {
    // Verify that the error is caught and handled correctly
    EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
   
  }
 
  
}


TEST(RealSenseErrorsChecker, TestErrorCatcherWriterPublisherInvalidValueError) {
    rs2::context ctx;
    rs2::device dev;
    ASSERT_NO_THROW(dev = ctx.query_devices()[0]);
    std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";
    auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");


  // Simulate an invalid error
  try {
    dev.hardware_reset();
  } catch (rs2::invalid_value_error& e) {
    // Verify that the error is caught and handled correctly
    EXPECT_EQ(checker->error_catcher_writer_publisher(e), EXIT_FAILURE);
    
  }
  
   
}





int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  // Initialize ROS2

  rclcpp::init(argc, argv);
  std::string path = "/home/belem/Desktop/CMR/realsense_errors_checker_pkg/error_log/realsense_errors_log.txt";

  auto checker = std::make_shared<realsenseEerorschecker::RealSenseErrorsChecker>("test_node", path, "test_topic");

  checker->start_msg_boot_up_msg();
  int ret = RUN_ALL_TESTS();

  // Shutdown ROS2
  rclcpp::spin(checker);
  rclcpp::shutdown();

 

  return ret;

}