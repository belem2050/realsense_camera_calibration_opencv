#include <ctime>                // for time
#include <fstream>              // ffor .txt files
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"


namespace realsenseHealthChecker{
    class RealsenseHealthChecker :public rclcpp::Node
    {
    private:
        std::string file_name_full_path_ ;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_error_msg_;
        rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_diagnostic_;

        /// @brief The callback function that listen to /diagnostic topic msg
        /// @param msg msg listening for
        void handleDiagnosticMsg(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

        /// @brief This static function is about writing a message in a file given by its path and name
        /// @param file_name_full_path  The error log file full path and name
        /// @param msg The message to be written
        /// @return True if succeeded and False if not
        static bool write_file(const std::string& file_name_full_path, const std::string& msg);

        /// @brief Creates a string error message which should be written in the error_log file and then publish on "error_topic_name"
        /// @param msg The error message
        /// @param error_code error code
        /// @param source  error source
        
        /// @return String error message to be written in the log file
        static std::string create_error_file_msg(const std::string& msg, const std::string& error_code, const std::string& source);
    
        /// @brief write the boot-up message when Realsense 3D camera is launching
        /// @param 
        /// @return 
        void start_msg_boot_up_msg();

        

    public:
        /// @brief Destructor
        ~RealsenseHealthChecker();
        //Constructor
        RealsenseHealthChecker(const std::string& node_name);
       
    };
    
};


