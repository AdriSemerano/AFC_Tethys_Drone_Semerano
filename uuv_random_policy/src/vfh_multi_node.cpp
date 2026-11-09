#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

#define RAD2DEG(x) ((x) * 180.0 / M_PI)

struct Waypoint { double x, y, z; };
struct Candidate { double yaw; double pitch; double cost; };

class VFHMultiNode : public rclcpp::Node
{
public:
    VFHMultiNode() : Node("vfh_multi_node"), rng_(std::random_device{}())
    {
        this->declare_parameter<std::string>("robot_name", "tethys_0");
        this->get_parameter("robot_name", robot_name_);

        std::string thrust_topic = "/model/" + robot_name_ + "/joint/propeller_joint/cmd_thrust";
        std::string fin_v_topic = "/model/" + robot_name_ + "/joint/vertical_fins_joint/fin_0/cmd_pos";
        std::string fin_h_topic = "/model/" + robot_name_ + "/joint/horizontal_fins_joint/fin_0/cmd_pos";
        
        std::string sonar_topic = "/" + robot_name_ + "/sonar_3d/points";
        std::string alt_topic = "/" + robot_name_ + "/altimeter";
        std::string odom_topic = "/model/" + robot_name_ + "/odometry";
        std::string bumper_topic = "/" + robot_name_ + "/bumper";

        thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>(thrust_topic, 10);
        fin_v_pub_ = this->create_publisher<std_msgs::msg::Float64>(fin_v_topic, 10);
        fin_h_pub_ = this->create_publisher<std_msgs::msg::Float64>(fin_h_topic, 10);

        sonar_3d_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            sonar_topic, 10, std::bind(&VFHMultiNode::sonar_3d_cb, this, _1));
        altimeter_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            alt_topic, 10, std::bind(&VFHMultiNode::altimeter_cb, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&VFHMultiNode::odom_cb, this, _1));
        bumper_sub_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>(
            bumper_topic, 10, std::bind(&VFHMultiNode::bumper_cb, this, _1));

        std::string home_dir = getenv("HOME");
        load_waypoints(home_dir + "/VFH_Project/wp_" + robot_name_ + ".txt");
        spherical_histogram_.assign(num_yaw_sectors_, std::vector<double>(num_pitch_sectors_, 0.0));
        
        load_rl_weights(home_dir + "/VFH_Project/vfh_rl_weights.txt"); 
        
        control_timer_ = this->create_wall_timer(100ms, std::bind(&VFHMultiNode::control_loop, this));
        telemetry_timer_ = this->create_wall_timer(500ms, std::bind(&VFHMultiNode::write_telemetry, this));
    }

private:
    std::string robot_name_;
    bool is_active_ = true; 
    std::string test_result_ = "ACTIVE"; 
    double current_dist_to_wp_ = 999.0;

    const double MAX_FIN_ANGLE = 0.2618;  
    const double MIN_THRUST = -40.0;      
    const double MAX_THRUST = 30.0;       

    double safe_dist_ = 20.0;             
    int obstacle_expansion_window_ = 4;   
    double vfh_a_ = 15.0;         
    double BETA_TEMP_ = 2.0; 
    double W_TARGET_DIR_ = 6.0;           
    double W_PITCH_DIR_  = 6.0;           
    double W_OBS_        = 8.0;           
    double W_PREV_DIR_   = 1.5;           
    double W_PITCH_STABILITY_ = 0.5;      
    double W_THRUST_     = 5.0;           

    double Kp_yaw_ = 1.5;         
    double Kp_pitch_ = 1.5;       
    double SMOOTH_FACTOR = 0.35; 

    double current_x_, current_y_, current_z_, current_roll_, current_pitch_, current_yaw_;
    double current_altitude_ = 999.0;
    bool has_odom_ = false;
    
    std::vector<Waypoint> waypoints_;
    size_t current_wp_index_ = 0;
    const double waypoint_tolerance_ = 4.0;
    const int num_yaw_sectors_ = 72;   
    const int num_pitch_sectors_ = 36; 
    
    double prev_chosen_yaw_ = 0.0;
    double prev_chosen_pitch_ = 0.0; 
    double prev_cmd_fin_v_ = 0.0;
    double prev_cmd_fin_h_ = 0.0;    
    
    std::vector<std::vector<double>> spherical_histogram_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    bool has_cloud_ = false;
    std::mt19937 rng_; 

    void load_waypoints(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) { waypoints_ = {{0.0, 0.0, 40.0}}; return; }
        waypoints_.clear();
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Waypoint wp;
            if (iss >> wp.x >> wp.y >> wp.z) waypoints_.push_back(wp);
        }
    }

    void load_rl_weights(const std::string& path) {
        std::ifstream file(path);
        if (file.is_open()) {
            double raw_expansion;
            file >> safe_dist_ >> raw_expansion >> W_TARGET_DIR_ >> W_PITCH_DIR_ >> W_OBS_ >> W_PREV_DIR_ >> W_PITCH_STABILITY_ >> W_THRUST_;
            obstacle_expansion_window_ = static_cast<int>(std::round(raw_expansion));
        }
    }

    void log_result() {
        std::ofstream res_file("/tmp/result_" + robot_name_ + ".txt");
        res_file << test_result_ << "\n";
        res_file.close();
        write_telemetry();
    }

    void write_telemetry() {
        if (!has_odom_) return;
        std::ofstream stat_file("/tmp/status_" + robot_name_ + ".txt");
        stat_file << test_result_ << " " << current_dist_to_wp_ << " " 
                  << current_x_ << " " << current_y_ << " " << current_z_ << "\n";
        stat_file.close();
    }

    void bumper_cb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { 
        if (!is_active_ || msg->contacts.empty()) return;
        
        is_active_ = false; 
        bool hit_other_drone = false;
        
        for (const auto& contact : msg->contacts) {
            std::string c1 = contact.collision1.name;
            std::string c2 = contact.collision2.name;
            if ((c1.find("tethys") != std::string::npos && c1.find(robot_name_) == std::string::npos) ||
                (c2.find("tethys") != std::string::npos && c2.find(robot_name_) == std::string::npos)) {
                hit_other_drone = true;
                break;
            }
        }
        
        test_result_ = hit_other_drone ? "CRASH_DRONE" : "CRASH_OBSTACLE";
        log_result();
        publish_commands(0.0, 0.0, 0.0);
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(current_roll_, current_pitch_, current_yaw_);
        has_odom_ = true;
    }

    void altimeter_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) { 
        if(!msg->ranges.empty()) {
            double raw_alt = msg->ranges[0];
            current_altitude_ = (std::isinf(raw_alt)) ? 999.0 : raw_alt * std::abs(std::cos(current_pitch_)) * std::abs(std::cos(current_roll_));
        } 
    }

    void sonar_3d_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        latest_cloud_ = msg;
        has_cloud_ = true;
    }

    void control_loop()
    {
        if (!has_odom_ || !has_cloud_) return;
        
        if (!is_active_) {
            publish_commands(0.0, 0.0, 0.0);
            return;
        }

        if (latest_cloud_->data.empty() || latest_cloud_->width == 0) return;

        Waypoint wp = waypoints_[current_wp_index_];
        double dx = wp.x - current_x_, dy = wp.y - current_y_, dz = wp.z - current_z_;
        current_dist_to_wp_ = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (current_dist_to_wp_ < waypoint_tolerance_) {
            is_active_ = false;
            test_result_ = "SUCCESS";
            log_result();
            publish_commands(0.0, 0.0, 0.0);
            return;
        }

        double nose_yaw = std::fmod(current_yaw_ + M_PI + 4.0*M_PI, 2.0*M_PI);
        double target_yaw_global = std::atan2(dy, dx);
        double local_target_yaw = std::atan2(std::sin(target_yaw_global - nose_yaw), std::cos(target_yaw_global - nose_yaw));
        double target_pitch_global = std::atan2(dz, std::hypot(dx, dy));
        double local_target_pitch = target_pitch_global - current_pitch_;

        for (int y = 0; y < num_yaw_sectors_; ++y) std::fill(spherical_histogram_[y].begin(), spherical_histogram_[y].end(), 0.0);

        double min_obs_dist = 999.0;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_cloud_, "x"), iter_y(*latest_cloud_, "y"), iter_z(*latest_cloud_, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x, y = *iter_y, z = *iter_z;
            if (std::isnan(x) || std::isnan(y) || std::isnan(z)) continue;
            
            double raw_d = std::sqrt(x*x + y*y + z*z);
            if (std::isinf(raw_d) || raw_d < 0.5 || raw_d > safe_dist_) continue;

            double y_unroll = y * std::cos(current_roll_) - z * std::sin(current_roll_);
            double z_unroll = y * std::sin(current_roll_) + z * std::cos(current_roll_);
            double x_stab = x * std::cos(current_pitch_) + z_unroll * std::sin(current_pitch_);
            double z_stab = -x * std::sin(current_pitch_) + z_unroll * std::cos(current_pitch_);
            double y_stab = y_unroll;

            double d = std::sqrt(x_stab*x_stab + y_stab*y_stab + z_stab*z_stab);
            if (d < min_obs_dist) min_obs_dist = d; 

            double pt_yaw = std::atan2(y_stab, x_stab);
            double pt_pitch = std::atan2(z_stab, std::hypot(x_stab, y_stab));

            int y_idx = static_cast<int>(std::fmod(pt_yaw + M_PI + 2.0*M_PI, 2.0*M_PI) / (2.0*M_PI / num_yaw_sectors_)) % num_yaw_sectors_;
            int p_idx = static_cast<int>(std::fmod(pt_pitch + M_PI/2.0, M_PI) / (M_PI / num_pitch_sectors_));
            p_idx = std::clamp(p_idx, 0, num_pitch_sectors_ - 1);

            double mag = vfh_a_ - ((vfh_a_ / safe_dist_) * d);
            spherical_histogram_[y_idx][p_idx] = std::max(spherical_histogram_[y_idx][p_idx], mag);
        }

        if (current_altitude_ < 15.0) {
            double alt_mag = vfh_a_ - (vfh_a_ / 15.0 * current_altitude_);
            for (int y = 0; y < num_yaw_sectors_; ++y) {
                for (int p = 0; p < num_pitch_sectors_ / 2; ++p) 
                    spherical_histogram_[y][p] = std::max(spherical_histogram_[y][p], alt_mag + 20.0);
            }
        }

        std::vector<std::vector<double>> expanded(num_yaw_sectors_, std::vector<double>(num_pitch_sectors_, 0.0));
        int win = obstacle_expansion_window_;
        for (int y = 0; y < num_yaw_sectors_; ++y) {
            for (int p = 0; p < num_pitch_sectors_; ++p) {
                double max_m = 0.0;
                for (int wy = -win; wy <= win; ++wy) {
                    for (int wp = -win; wp <= win; ++wp) {
                        max_m = std::max(max_m, spherical_histogram_[(y+wy+num_yaw_sectors_)%num_yaw_sectors_][std::clamp(p+wp,0,num_pitch_sectors_-1)]);
                    }
                }
                expanded[y][p] = max_m;
            }
        }
        spherical_histogram_ = expanded;

        std::vector<Candidate> candidates;
        double min_G = std::numeric_limits<double>::max();

        for (int y = 0; y < num_yaw_sectors_; ++y) {
            for (int p = 0; p < num_pitch_sectors_; ++p) {
                double c_yaw = (y * (2.0*M_PI / num_yaw_sectors_)) - M_PI;
                double c_pitch = (p * (M_PI / num_pitch_sectors_)) - M_PI/2.0;
                double obs = spherical_histogram_[y][p];

                double G = (W_TARGET_DIR_ * std::abs(std::atan2(std::sin(c_yaw - local_target_yaw), std::cos(c_yaw - local_target_yaw)))) + 
                           (W_PITCH_DIR_ * std::abs(c_pitch - local_target_pitch)) + 
                           (W_OBS_ * obs) + 
                           (W_PREV_DIR_ * std::abs(std::atan2(std::sin(c_yaw - prev_chosen_yaw_), std::cos(c_yaw - prev_chosen_yaw_)))) +
                           (W_PITCH_STABILITY_ * std::abs(c_pitch)); 

                candidates.push_back({c_yaw, c_pitch, G});
                if (G < min_G) min_G = G;
            }
        }

        double sum_weights = 0.0;
        std::vector<double> weights(candidates.size());
        for (size_t i = 0; i < candidates.size(); ++i) {
            weights[i] = std::exp(-BETA_TEMP_ * (candidates[i].cost - min_G));
            sum_weights += weights[i];
        }

        std::uniform_real_distribution<double> dist_rnd(0.0, sum_weights);
        double random_val = dist_rnd(rng_);
        double cumulative = 0.0;
        
        double chosen_local_yaw = local_target_yaw;
        double chosen_local_pitch = local_target_pitch;
        double best_G = min_G; 

        for (size_t i = 0; i < candidates.size(); ++i) {
            cumulative += weights[i];
            if (random_val <= cumulative) {
                chosen_local_yaw = candidates[i].yaw;
                chosen_local_pitch = candidates[i].pitch;
                best_G = candidates[i].cost;
                break;
            }
        }
        prev_chosen_yaw_ = chosen_local_yaw; prev_chosen_pitch_ = chosen_local_pitch;

        double body_yaw_err = chosen_local_yaw;
        double body_pitch_err = chosen_local_pitch - current_pitch_;

        double thrust = std::clamp(MIN_THRUST + (best_G * W_THRUST_), MIN_THRUST, MAX_THRUST);
        double comp_y = (body_yaw_err * std::cos(current_roll_)) - (body_pitch_err * std::sin(current_roll_));
        double comp_p = (body_pitch_err * std::cos(current_roll_)) + (body_yaw_err * std::sin(current_roll_));

        double raw_v = (thrust < 0) ? -(comp_y * Kp_yaw_) : (comp_y * Kp_yaw_);
        double raw_h = comp_p * Kp_pitch_;

        double cmd_v = prev_cmd_fin_v_ + SMOOTH_FACTOR * (std::clamp(raw_v, -MAX_FIN_ANGLE, MAX_FIN_ANGLE) - prev_cmd_fin_v_);
        double cmd_h = prev_cmd_fin_h_ + SMOOTH_FACTOR * (std::clamp(raw_h, -MAX_FIN_ANGLE, MAX_FIN_ANGLE) - prev_cmd_fin_h_);
        
        prev_cmd_fin_v_ = cmd_v; prev_cmd_fin_h_ = cmd_h;
        publish_commands(cmd_v, cmd_h, thrust);
    }

    void publish_commands(double v, double h, double t) {
        std_msgs::msg::Float64 vm, hm, tm;
        vm.data = v; hm.data = h; tm.data = t;
        fin_v_pub_->publish(vm); fin_h_pub_->publish(hm); thrust_pub_->publish(tm);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub_, fin_v_pub_, fin_h_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_3d_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr altimeter_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr bumper_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_, telemetry_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VFHMultiNode>());
    rclcpp::shutdown();
    return 0;
}
