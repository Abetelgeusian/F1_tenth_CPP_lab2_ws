#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <vector>
#include <limits>
using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",1, std::bind(&Safety::scan_callback, this, _1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom",1,std::bind(&Safety::drive_callback,this,_1));
        drive_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive",1);
        
    }

private:
    double speed = 0.0;
    float f_max = std::numeric_limits<float>::infinity();
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        this->speed = msg->twist.twist.linear.x;
        // RCLCPP_INFO(this->get_logger(), "Speed is: '%f'", this->speed);
    }

    std::vector<float> range_change_rate(const std::vector<float>& angles){ // experiment here! instead of angle vector see if you can access the angles stream directly.
        std::vector<float> result;
        for( float i : angles){
            result.push_back(this->speed * std::cos(i));
        }
        return result;
    }

    std::vector<float> ttc_calc(const std::vector<float>& r_dot, const std::vector<float>& r){
        std::vector<float> result;
        for(int i=0; i<r_dot.size(); i++){
            if(r_dot.at(i)>0 && !std::isinf(r.at(i)) && !std::isnan(r.at(i))){
                result.push_back(r.at(i)/r_dot.at(i));
            }
            else{
                result.push_back(300.0);
            }
        }
        return result;
    }



    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        std::vector<float> angles;
        float theta = scan_msg->angle_min;
        while(theta <= scan_msg->angle_max){
            angles.push_back(theta);
            theta += scan_msg->angle_increment;
        }

        std::vector<float> r_dot = range_change_rate(angles);
        /// TODO: calculate TTC
        std::vector<float> ttc = ttc_calc(r_dot, scan_msg->ranges);
        /// TODO: publish drive/brake message
        int n = ttc.size();
        float min  = 1000;
        for (int i=0;i<n;i++){
            if(ttc.at(i) < min){
                min = ttc.at(i);
            }
        }

        if(min < 1.25){            
            // RCLCPP_INFO(this->get_logger(), "Moving!");
            RCLCPP_INFO(this->get_logger(), "Stopeed because TTC is '%f'", min);
            auto brake = ackermann_msgs::msg::AckermannDriveStamped();
            brake.header.stamp = this->now();
            brake.drive.speed = 0.0;
            this->drive_pub->publish(brake);
        }
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}