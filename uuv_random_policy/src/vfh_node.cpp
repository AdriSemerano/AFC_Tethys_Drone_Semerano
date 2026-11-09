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

class VFHNodeRL : public rclcpp::Node
{
public:
    VFHNodeRL() : Node("vfh_node_rl"), rng_(std::random_device{}())
    {
        thrust_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/tethys/joint/propeller_joint/cmd_thrust", 10);
        fin_v_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/tethys/joint/vertical_fins_joint/fin_0/cmd_pos", 10);
        fin_h_pub_ = this->create_publisher<std_msgs::msg::Float64>("/model/tethys/joint/horizontal_fins_joint/fin_0/cmd_pos", 10);

        sonar_3d_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/tethys/sonar_3d/points", 10, std::bind(&VFHNodeRL::sonar_3d_cb, this, _1));
            
        altimeter_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/tethys/altimeter", 10, std::bind(&VFHNodeRL::altimeter_cb, this, _1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/tethys/odometry", 10, std::bind(&VFHNodeRL::odom_cb, this, _1));
        bumper_sub_ = this->create_subscription<ros_gz_interfaces::msg::Contacts>("/tethys/bumper", 10, std::bind(&VFHNodeRL::bumper_cb, this, _1));

        load_waypoints("/home/adriano/VFH_Project/current_waypoints.txt");
        spherical_histogram_.assign(num_yaw_sectors_, std::vector<double>(num_pitch_sectors_, 0.0));
        
        load_rl_weights(); 
        
        control_timer_ = this->create_wall_timer(100ms, std::bind(&VFHNodeRL::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "VFH Node initialized with RL weights and unlocked dynamics.");
    }

private:
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
    double SMOOTH_FACTOR = 0.35; // Smoothed to prevent water drag skidding/derapate

    double current_x_, current_y_, current_z_, current_roll_, current_pitch_, current_yaw_;
    double current_altitude_ = 999.0;
    bool has_odom_ = false;
    bool bumper_hit_event_ = false;
    int collision_counter_ = 0;
    int backoff_counter_ = 0; 
    std::string current_state_ = "NAVIGATING";
    std::string obs_direction_log_ = "CLEAR";
    
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
        if (!file.is_open()) {
            waypoints_ = {{0.0, 0.0, 40.0}}; return;
        }
        waypoints_.clear();
        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Waypoint wp;
            if (iss >> wp.x >> wp.y >> wp.z) waypoints_.push_back(wp);
        }
    }

    void load_rl_weights() {
        std::string home_dir = getenv("HOME");
        std::string path = home_dir + "/VFH_Project/vfh_rl_weights.txt";
        std::ifstream file(path);
        if (file.is_open()) {
            double raw_expansion;
            file >> safe_dist_ >> raw_expansion >> W_TARGET_DIR_ >> W_PITCH_DIR_ >> W_OBS_ >> W_PREV_DIR_ >> W_PITCH_STABILITY_ >> W_THRUST_;
            obstacle_expansion_window_ = static_cast<int>(std::round(raw_expansion));
        }
    }

    void bumper_cb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg) { 
        if (!msg->contacts.empty()) bumper_hit_event_ = true;
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
        if (latest_cloud_->data.empty() || latest_cloud_->width == 0) return;

        if (current_wp_index_ >= waypoints_.size()) {
            publish_commands(0.0, 0.0, 0.0);
            current_state_ = "MISSION COMPLETED";
            return;
        }

        bool impacted_now = bumper_hit_event_;
        if (bumper_hit_event_) { 
            collision_counter_++; 
            bumper_hit_event_ = false; 
            backoff_counter_ = 10;
        }
        
        Waypoint wp = waypoints_[current_wp_index_];
        double dx = wp.x - current_x_, dy = wp.y - current_y_, dz = wp.z - current_z_;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < waypoint_tolerance_) {
            current_wp_index_++; 
            return;
        }

        if (backoff_counter_ > 0) {
            backoff_counter_--;
            current_state_ = "COLLISION: BACKOFF TIMER";
            prev_cmd_fin_v_ = 0.0; prev_cmd_fin_h_ = 0.0;
            publish_commands(0.0, 0.0, 0.0);
            return;
        }

        double nose_yaw = std::fmod(current_yaw_ + M_PI + 4.0*M_PI, 2.0*M_PI);
        double target_yaw_global = std::atan2(dy, dx);
        double local_target_yaw = std::atan2(std::sin(target_yaw_global - nose_yaw), std::cos(target_yaw_global - nose_yaw));
        
        double target_pitch_global = std::atan2(dz, std::hypot(dx, dy));
        double local_target_pitch = target_pitch_global - current_pitch_;

        for (int y = 0; y < num_yaw_sectors_; ++y) std::fill(spherical_histogram_[y].begin(), spherical_histogram_[y].end(), 0.0);

        bool obs_up = false, obs_down = false, obs_left = false, obs_right = false, obs_front = false;
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

            if (d < safe_dist_) {
                if (pt_pitch > 0.15) obs_up = true; else if (pt_pitch < -0.15) obs_down = true;
                if (pt_yaw > 0.2) obs_left = true; else if (pt_yaw < -0.2) obs_right = true;
                if (std::abs(pt_yaw) < 0.2 && std::abs(pt_pitch) < 0.15) obs_front = true;
            }

            int y_idx = static_cast<int>(std::fmod(pt_yaw + M_PI + 2.0*M_PI, 2.0*M_PI) / (2.0*M_PI / num_yaw_sectors_)) % num_yaw_sectors_;
            int p_idx = static_cast<int>(std::fmod(pt_pitch + M_PI/2.0, M_PI) / (M_PI / num_pitch_sectors_));
            p_idx = std::clamp(p_idx, 0, num_pitch_sectors_ - 1);

            double mag = vfh_a_ - ((vfh_a_ / safe_dist_) * d);
            spherical_histogram_[y_idx][p_idx] = std::max(spherical_histogram_[y_idx][p_idx], mag);
        }

        if (min_obs_dist < safe_dist_) {
            obs_direction_log_ = std::string(obs_front ? "[FRONT] " : "") + 
                                 (obs_up ? "[TOP] " : "") + 
                                 (obs_down ? "[BOT] " : "") + 
                                 (obs_left ? "[LEFT] " : "") + 
                                 (obs_right ? "[RIGHT] " : "");
            char dist_str[32];
            snprintf(dist_str, sizeof(dist_str), "(Dist: %.1fm)", min_obs_dist);
            obs_direction_log_ += dist_str;
        } else {
            obs_direction_log_ = "CLEAR";
        }

        bool floor_danger = false;
        if (current_altitude_ < 15.0) {
            floor_danger = true;
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

        current_state_ = floor_danger ? "FLOOR AVOIDANCE" : ((min_obs_dist < safe_dist_) ? "OBSTACLE AVOIDANCE" : "NAVIGATING");

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
        
        prev_cmd_fin_v_ = cmd_v; 
        prev_cmd_fin_h_ = cmd_h;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "\n--- TETHYS MONITORING ---\n"
            "STATE:     [%s]\n"
            "TARGET:    WP %zu/%zu | DIST: %.1f m\n"
            "ATTITUDE:  Roll: %.1f° | Pitch: %.1f°\n"
            "SENSORS:   Alt: %.1f m | Obstacles: %s\n"
            "TARGET R:  Loc_Yaw: %.1f° | Loc_Pitch: %.1f°\n"
            "DECISION:  Cost G: %.2f | VFH_Yaw: %.1f° | VFH_Pitch: %.1f°\n"
            "COMMANDS:  Thrust: %.1f | Fin_V: %.3f | Fin_H: %.3f\n"
            "BUMPER:    %s (Tot: %d)\n"
            "-------------------------",
            current_state_.c_str(), current_wp_index_ + 1, waypoints_.size(), dist,
            RAD2DEG(current_roll_), RAD2DEG(current_pitch_), current_altitude_, obs_direction_log_.c_str(),
            RAD2DEG(local_target_yaw), RAD2DEG(local_target_pitch),
            best_G, RAD2DEG(chosen_local_yaw), RAD2DEG(chosen_local_pitch),
            thrust, cmd_v, cmd_h, (impacted_now?"[!!! IMPACT !!!]":"[CLEAR]"), collision_counter_);

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
    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VFHNodeRL>());
    rclcpp::shutdown();
    return 0;
}
