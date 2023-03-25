#include "../include/realsense2_health_checker.hpp"

#include <nlohmann/json.hpp>
using json = nlohmann::json;
using namespace realsenseHealthChecker;
#define  ERROR  0
#define  STALE 3

RealsenseHealthChecker::RealsenseHealthChecker(const std::string& node_name)
: Node(node_name)
{   
    // ROS Params
    std::string error_file_directory, error_file_name, error_file_extension, topic_name;
    this->declare_parameter<std::string>("error_file_directory", "/home/ros/.robot/errorlogs");
    this->declare_parameter<std::string>("error_file_name", "realsense_errors");
    this->declare_parameter<std::string>("error_file_extension", "txt");
    this->declare_parameter<std::string>("topic_name", "/topic");
    this->get_parameter("error_file_directory", error_file_directory);
    this->get_parameter("error_file_name", error_file_name);
    this->get_parameter("error_file_extension", error_file_extension);
    this->get_parameter("topic_name", topic_name);

    // Init error file name with the current date
    const auto time = std::chrono::system_clock::now();
    std::time_t today_time = std::chrono::system_clock::to_time_t(time);

    char date[std::size("yyyy-mm-dd")];
    std::strftime(std::data(date), std::size(date), "%F", std::gmtime(&today_time));

    this->file_name_full_path_ = error_file_directory + std::string("/") + error_file_name + std::string("_") + std::string(date) + std::string(".") + error_file_extension;
    std::cout << this->file_name_full_path_<< std::endl;
    this->start_msg_boot_up_msg();
    this->publisher_error_msg_ = this->create_publisher<std_msgs::msg::String>(topic_name, 10);
    this->subscriber_diagnostic_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 10,std::bind(&RealsenseHealthChecker::handleDiagnosticMsg, this, std::placeholders::_1));
       
}

RealsenseHealthChecker::~RealsenseHealthChecker(){}


bool RealsenseHealthChecker::write_file(const std::string& file_name_full_path_, const std::string& msg)
    {
        std::ofstream file (file_name_full_path_, std::ofstream::out | std::ofstream::app);

        if (file.is_open()) {
            file << msg;
            file.close();
            return true;
        }

        return false;
    }    
     


void RealsenseHealthChecker::handleDiagnosticMsg(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
    if (msg->status.size() > 0) {
        std_msgs::msg::String error_msg;
       
        
     

            if(msg->status.size()>=3){

                if((msg->status[0].name == "camera: Temperatures") && (msg->status[0].level ==ERROR || msg->status[0].level ==STALE)){
                    RCLCPP_ERROR(this->get_logger(), "Error from Temperatures camera "); 
                    json error_json = {  
                        {"camera","Temperatures"},          
                        {"error code",msg->status[0].hardware_id},
                        {"message",msg->status[0].message}      
                    };
                    if(!write_file(this->file_name_full_path_,create_error_file_msg(error_json["message"],error_json["error code"],error_json["camera"]))){
                        RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path_.c_str()); ;  
                    }
                    error_msg.data =error_json.dump();
                    publisher_error_msg_->publish(error_msg);
                }

                if((msg->status[1].name == "camera: depth") && (msg->status[1].level ==ERROR|| msg->status[1].level ==STALE)){
                    RCLCPP_ERROR(this->get_logger(), "Error from depth camera "); 
                    json error_json = {  
                        {"camera","depth"},          
                        {"error code",msg->status[1].hardware_id},
                        {"message",msg->status[1].message}      
                    };
                    if(!write_file(this->file_name_full_path_,create_error_file_msg(error_json["message"],error_json["error code"],error_json["camera"]))){
                        RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path_.c_str()); ;  
                    }
                    error_msg.data =error_json.dump();
                    publisher_error_msg_->publish(error_msg);
                }

                if((msg->status[2].name == "camera: color") && (msg->status[2].level ==ERROR || msg->status[2].level ==STALE)){
                    RCLCPP_ERROR(this->get_logger(), "Error from color camera "); 
                    json error_json = {  
                        {"camera","color"},          
                        {"error code",msg->status[2].hardware_id},
                        {"message",msg->status[2].message}      
                    };
                    if(!write_file(this->file_name_full_path_,create_error_file_msg(error_json["message"],error_json["error code"],error_json["camera"]))){
                        RCLCPP_ERROR(this->get_logger(), "Error while writing into the file '%s'", file_name_full_path_.c_str()); ;  
                    }
                    error_msg.data =error_json.dump();
                    publisher_error_msg_->publish(error_msg);
                }
            }

        
   }            
    
}

 
void RealsenseHealthChecker::start_msg_boot_up_msg(){
        const auto time = std::chrono::system_clock::now();
        std::time_t today_time = std::chrono::system_clock::to_time_t(time);
        
        if (!write_file(this->file_name_full_path_, std::string("\n\n##############################################\n"
                                                        "##### RealSense start: ") + std::ctime(&today_time) +
                                            std::string("##############################################\n"))) {
            RCLCPP_ERROR(this->get_logger(), "Error while writing boot-up message into the file '%s'", file_name_full_path_.c_str()); ;  
        }

    }


std::string RealsenseHealthChecker::create_error_file_msg(const std::string& msg, const std::string& error_code, const std::string& source)
    {   
        std::string error_msg = std::string("\n========= New error received")  +  std::string(" =========\n");  
        const auto time = std::chrono::system_clock::now();
        std::time_t today_time = std::chrono::system_clock::to_time_t(time);
        error_msg.append(std::string("- time: ") + std::ctime(&today_time)); 
        error_msg.append(std::string("- Error source: ") + source + std::string("\n"));
        error_msg.append(std::string("- Error code: ") + error_code + std::string("\n"));
        error_msg.append(std::string("- Error message: ") + msg + std::string("\n"));
        
        return error_msg;

    }


 