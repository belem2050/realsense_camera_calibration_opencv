#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <ctime>                // for time
#include <fstream>              // ffor .txt files
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

/// @brief A class to catch and log all possible errors that could occur during realsense 3D camera work.

namespace realsenseEerorschecker{

    /*  
    Class that handles all occured errors in in a log file
    */
    class RealSenseErrorsChecker : public rclcpp::Node
    {
    private:
        // 
    
        const std::string& file_name_full_path ;
        //const std::string& node_name;
        const std::string& error_topic_name_;
        //
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_error_;


        /// @brief This static function is about writing a message in a file given by its path and name
        /// @param file_name_full_path  The error log file full path and name
        /// @param msg The message to be written
        /// @return True if succeeded and False if not
        static bool write_file(const std::string& file_name_full_path, const std::string& msg);


        /// @brief Creates a string error message which should be written in the error_log file and then publish on "error_topic_name"
        /// @param msg The error message
        /// @param error_code error code
        /// @param source  error source
        /// @param type  error type
        /// @return String error message to be written in the log file
        static std::string create_error_file_msg(const std::string& msg, const std::string& error_code, const std::string& source, const rs2_exception_type& type);

    public:
        /// @brief Constructor that takes in args the full path and ane of the log errors file
        /// @param file_name_full_path the full path name

        /// @brief 
        /// @param file_name_full_path 
        /// @param node_name 
        /// @param error_topic_name 
        RealSenseErrorsChecker(const std::string& node_name, const std::string& file_name_full_path, const std::string& error_topic_name);   

        /// @brief Destructor
        ~RealSenseErrorsChecker();

        ////////////////////////

        std::string get_path(){return file_name_full_path;};
        std::string get_error_topic_name(){return error_topic_name_;};
        
        /// @brief write the boot-up message when Realsense 3D camera is launching
        /// @param 
        /// @return 
        void start_msg_boot_up_msg();

        /// @brief Write error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const std::exception& e);

        /// @brief Write error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE code
        int error_catcher_writer_publisher(const rs2::error& e);

        /// @brief Write backend_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::backend_error& e);

        /// @brief Write invalid_value_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::invalid_value_error& e);

        /// @brief Write recoverable_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::recoverable_error& e);

        /// @brief Write unrecoverable_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::unrecoverable_error& e);

        /// @brief Write not_implemented_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::not_implemented_error& e);

        /// @brief Write camera_disconnected_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::camera_disconnected_error& e);

        /// @brief Write wrong_api_call_sequence_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::wrong_api_call_sequence_error& e);

        /// @brief Write device_in_recovery_mode_error occured and caught in the catch(..) in the error_log file and then publish on "error_topic_name"
        /// @param e error occured in the catch
        /// @return an EXIT_FAILURE CODE
        int error_catcher_writer_publisher(const rs2::device_in_recovery_mode_error& e);    

    };

    

    /*
        IMPLEMENTATION
    */
    RealSenseErrorsChecker::RealSenseErrorsChecker(const std::string& node_name_r, const std::string& file_name_full_path_r,const std::string& error_topic_name_r)
    : rclcpp::Node(node_name_r),file_name_full_path(file_name_full_path_r), error_topic_name_(error_topic_name_r)
    
    {
        publisher_error_ = this->create_publisher<std_msgs::msg::String>(error_topic_name_, 10);
        //this->start_msg_boot_up_msg();
    }

    
    RealSenseErrorsChecker::~RealSenseErrorsChecker()
    {
    }

    bool RealSenseErrorsChecker::write_file(const std::string& file_name_full_path, const std::string& msg)
    {
        std::ofstream file (file_name_full_path, std::ofstream::out | std::ofstream::app);

        if (file.is_open()) {
            file << msg;
            file.close();
            return true;
        }

        return false;
    }

    std::string RealSenseErrorsChecker::create_error_file_msg(const std::string& msg, const std::string& error_code, const std::string& source, const rs2_exception_type& type)
    {   
        std::string error_msg = std::string("\n========= New error received")  +  std::string(" =========\n");  
        const auto time = std::chrono::system_clock::now();
        std::time_t today_time = std::chrono::system_clock::to_time_t(time);
        error_msg.append(std::string("- time: ") + std::ctime(&today_time)); 
        error_msg.append(std::string("- Error code: ") + error_code + std::string("\n"));
        error_msg.append(std::string("- Error source: ") + source + std::string("\n"));
        error_msg.append(std::string("- Error type: ") + rs2_exception_type_to_string(type) + std::string("\n"));
        error_msg.append(std::string("- Error message: ") + msg + std::string("\n"));
        
        return error_msg;

    }
    

    void RealSenseErrorsChecker::start_msg_boot_up_msg(){
        const auto time = std::chrono::system_clock::now();
        std::time_t today_time = std::chrono::system_clock::to_time_t(time);
        
        if (!write_file(this->file_name_full_path, std::string("\n\n##############################################\n"
                                                        "##### RealSense start: ") + std::ctime(&today_time) +
                                            std::string("##############################################\n"))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing boot-up message into the file '%s'", file_name_full_path.c_str()); ;  
        }

    }


    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::error& e){
        
        RCLCPP_ERROR(this->get_logger(), "Error caught %s", e.what()); ; 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();             
        publisher_error_->publish(msg);

        return EXIT_FAILURE;
    }
        
    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::backend_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }

        auto msg =std_msgs::msg::String();
        msg.data = e.what();       
        publisher_error_->publish(msg);

        return EXIT_FAILURE;
    }
        

    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::recoverable_error& e){
        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what());
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        return EXIT_FAILURE;
    }
    
    

    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::invalid_value_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        
        return EXIT_FAILURE;
    }
        

    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::not_implemented_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        return EXIT_FAILURE;
    }
        

    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::camera_disconnected_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        return EXIT_FAILURE;
    }
        

    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::wrong_api_call_sequence_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        return EXIT_FAILURE;
    }

     
    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::device_in_recovery_mode_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        return EXIT_FAILURE;
    }


    int RealSenseErrorsChecker::error_catcher_writer_publisher(const rs2::unrecoverable_error& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        if (!write_file(this->file_name_full_path,create_error_file_msg(e.what(), e.get_failed_args(), e.get_failed_function(),e.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
    
        return EXIT_FAILURE;
    }
        

    int RealSenseErrorsChecker::error_catcher_writer_publisher(const std::exception& e){

        RCLCPP_ERROR(this->get_logger(), "Error caught : %s", e.what()); 
        rs2::error error(e.what());
        if (!this->write_file(this->file_name_full_path,this->create_error_file_msg(e.what(),error.get_failed_args(), error.get_failed_function(),error.get_type()))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path.c_str()); ;  
        }
        auto msg =std_msgs::msg::String();
        msg.data = e.what();         
        publisher_error_->publish(msg);
        return EXIT_FAILURE;
    }
};