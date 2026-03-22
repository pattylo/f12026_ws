#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

class Motor {
public:
    Motor(const std::vector<int>& servo_ids, const std::string& port = "/dev/ttyUSB0", int window_size = 10)
        : port_(port), servo_ids_(servo_ids), num_servos_(servo_ids.size()), 
          window_size_(window_size), sample_idx_(0), is_initialized_(false) {
        
        // Open serial port
        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ < 0) {
            throw std::runtime_error("Failed to open serial port: " + port);
        }
        
        // Configure serial port
        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        
        if (tcgetattr(fd_, &tty) != 0) {
            throw std::runtime_error("Error from tcgetattr");
        }
        
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        tty.c_iflag &= ~IGNBRK;                         // disable break processing
        tty.c_lflag = 0;                                // no signaling chars, no echo,
        tty.c_oflag = 0;                                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;                            // read doesn't block
        tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout
        
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
        tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
        tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            throw std::runtime_error("Error from tcsetattr");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "Initialized " << num_servos_ << " servos: ";
        for (int id : servo_ids_) std::cout << id << " ";
        std::cout << std::endl;
        
        // Enable torque for all servos
        for (int sid : servo_ids_) {
            enableTorque(sid);
        }
        
        // Stop all servos initially
        for (int sid : servo_ids_) {
            motorMode(sid, 0);
        }
        
        // Initialize arrays
        timestamps_.resize(window_size_, 0.0);
        unwrapped_angles_.resize(window_size_, std::vector<double>(num_servos_, 0.0));
        raw_angles_.resize(num_servos_, 0.0);
        prev_angles_.resize(num_servos_, 0.0);
        motor_vel_.resize(num_servos_, 0.0);
        target_speeds_.resize(num_servos_, 0);
    }
    
    ~Motor() {
        if (fd_ >= 0) {
            stop();
            close(fd_);
        }
    }
    
    void setMotorSpeed(const std::vector<int>& speeds) {
        for (size_t i = 0; i < speeds.size() && i < servo_ids_.size(); ++i) {
            int speed_clamped =0;//= std::clamp(speeds[i], -1000, 1000);
            target_speeds_[i] = speed_clamped;
            motorMode(servo_ids_[i], speed_clamped);
        }
    }
    
    void update() {
        readAngles();
        updateVelocity();
    }
    
    void stop() {
        for (int servo_id : servo_ids_) {
            motorMode(servo_id, 0);
        }
    }
    
    const std::vector<double>& getMotorVelocities() const {
        return motor_vel_;
    }

private:
    std::string port_;
    int fd_;
    std::vector<int> servo_ids_;
    size_t num_servos_;
    int window_size_;
    
    std::vector<double> timestamps_;
    std::vector<std::vector<double>> unwrapped_angles_;
    std::vector<double> raw_angles_;
    std::vector<double> prev_angles_;
    std::vector<double> motor_vel_;
    std::vector<int> target_speeds_;
    int sample_idx_;
    bool is_initialized_;
    
    std::vector<uint8_t> makePacket(uint8_t servo_id, uint8_t cmd, const std::vector<uint8_t>& params) {
        uint8_t length = 3 + params.size();
        std::vector<uint8_t> packet = {0x55, 0x55, servo_id, length, cmd};
        packet.insert(packet.end(), params.begin(), params.end());
        
        uint8_t checksum = 0;
        for (size_t i = 2; i < packet.size(); ++i) {
            checksum += packet[i];
        }
        checksum = ~checksum;
        packet.push_back(checksum);
        
        return packet;
    }
    
    void enableTorque(uint8_t servo_id) {
        auto packet = makePacket(servo_id, 31, {0x01});
        write(fd_, packet.data(), packet.size());
        tcdrain(fd_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    void motorMode(uint8_t servo_id, int speed) {
        speed = 0;//std::clamp(speed, -1000, 1000);
        
        uint16_t speed_unsigned;
        if (speed < 0) {
            speed_unsigned = 65536 + speed;  // Two's complement
        } else {
            speed_unsigned = speed;
        }
        
        uint8_t low_byte = speed_unsigned & 0xFF;
        uint8_t high_byte = (speed_unsigned >> 8) & 0xFF;
        
        auto packet = makePacket(servo_id, 29, {0x01, 0x00, low_byte, high_byte});
        write(fd_, packet.data(), packet.size());
        tcdrain(fd_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    void readAngles() {
        // Stub - requires position mode to read angles
        // For now, just keep previous values
    }
    
    void updateVelocity() {
        using namespace std::chrono;
        auto now = steady_clock::now();
        double current_time = duration_cast<duration<double>>(now.time_since_epoch()).count();
        timestamps_[sample_idx_] = current_time;
        
        for (size_t i = 0; i < num_servos_; ++i) {
            // Unwrap angle
            double angle_diff = raw_angles_[i] - prev_angles_[i];
            if (angle_diff > M_PI) {
                angle_diff -= 2 * M_PI;
            } else if (angle_diff < -M_PI) {
                angle_diff += 2 * M_PI;
            }
            
            if (is_initialized_) {
                int prev_idx = (sample_idx_ - 1 + window_size_) % window_size_;
                unwrapped_angles_[sample_idx_][i] = unwrapped_angles_[prev_idx][i] + angle_diff;
            } else {
                unwrapped_angles_[sample_idx_][i] = raw_angles_[i];
            }
            
            prev_angles_[i] = raw_angles_[i];
        }
        
        if (is_initialized_) {
            // Calculate velocity from window
            int oldest_idx = (sample_idx_ + 1) % window_size_;
            double dt = timestamps_[sample_idx_] - timestamps_[oldest_idx];
            if (dt > 0) {
                for (size_t i = 0; i < num_servos_; ++i) {
                    double dangle = unwrapped_angles_[sample_idx_][i] - unwrapped_angles_[oldest_idx][i];
                    motor_vel_[i] = dangle / dt;
                }
            }
        }
        
        sample_idx_ = (sample_idx_ + 1) % window_size_;
        if (sample_idx_ == 0) {
            is_initialized_ = true;
        }
    }
};


class ServoControlNode : public rclcpp::Node {
public:
    ServoControlNode() : Node("servo_control_node"), desired_manual_(2, 0.0) {
        // Initialize motor controller
        motor_ = std::make_unique<Motor>(std::vector<int>{1, 2}, "/dev/ttyUSB0");
        
        // Subscribe to RC input
        rc_sub_ = this->create_subscription<mavros_msgs::msg::RCIn>(
            "/mavros/rc/in",
            10,
            std::bind(&ServoControlNode::rcCallback, this, std::placeholders::_1)
        );
        
        // Timer for motor control updates
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ServoControlNode::controlLoop, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "ctrl via RC");
    }
    
    ~ServoControlNode() {
        shutdown();
    }
    
    void shutdown() {
        if (motor_) {
            motor_->stop();
        }
    }

private:
    std::unique_ptr<Motor> motor_;
    std::vector<double> desired_manual_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr rc_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rate_sub_;

    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::vector<double> differentialIK(const std::vector<double>& vel) {
        double v_fwd_cmd = vel[0];
        double v_turn_cmd = vel[1];
        
        double w_left = v_fwd_cmd - v_turn_cmd;
        double w_right = v_fwd_cmd + v_turn_cmd;
        
        return {-w_right, w_left};
    }
    
    void rcCallback(const mavros_msgs::msg::RCIn::SharedPtr msg) {
        // switch here
        if (msg->channels.size() > 6 && msg->channels[6] < 2000) {
            std::fill(desired_manual_.begin(), desired_manual_.end(), 0.0);
        } else if (msg->channels.size() > 1) {
            double k = 100.0;
            
            std::vector<double> vel = {
                -k * (msg->channels[1] - 1515.0) / (2015.0 - 1015.0) * 2.0,
                k * (msg->channels[0] - 1515.0) / (2015.0 - 1015.0) * 2.0
            };
            
            desired_manual_ = differentialIK(vel);
        }
    }
    
    void controlLoop() {
        std::vector<int> motor_speeds;
        for (double val : desired_manual_) {
            int speed =0;//= static_cast<int>(std::clamp(val * 10.0, -1000.0, 1000.0));
            motor_speeds.push_back(speed);
        }
        // motor_speeds = {100, 0};  // For testing
        // std::cout << motor_speeds[0] << " " << motor_speeds[1] << std::endl;
        motor_->setMotorSpeed(motor_speeds);
        motor_->update();
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ServoControlNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    node->shutdown();
    rclcpp::shutdown();
    
    return 0;
}
