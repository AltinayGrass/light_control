#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class LightControl : public rclcpp::Node
{
public:
    LightControl() : Node("light_control")
    {
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", 10, 
            std::bind(&LightControl::odom_callback, this, std::placeholders::_1));

        subscription_gpio_ = this->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
            "/gpio_controller/gpio_states", 10, 
            std::bind(&LightControl::gpio_callback, this, std::placeholders::_1));

        subscription_battery_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/bms_status", 10, 
            std::bind(&LightControl::battery_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
            "/gpio_controller/commands", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&LightControl::timer_callback, this));

        last_twist_x_ = 0.0;
        direction_ = 0;
        emg_ = 1;
        charging_ = false;
        battery_full_ = false;
        stop_counter_ = 0;
        last_stopped_ = false;
        last_emg_ = emg_;
        last_charging_ = charging_;
        last_battery_full_ = battery_full_;
        last_odom_time_ = this->now();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_twist_x_ = msg->twist.twist.linear.x;
        last_odom_time_ = this->now();
    }

    void gpio_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
    {
        int new_emg = 1;

        for (const auto &values : msg->interface_values)
        {
            int di1_index = -1, di9_index = -1;
            for (size_t j = 0; j < values.interface_names.size(); j++)
            {
                if (values.interface_names[j] == "di.1") di1_index = j;
                if (values.interface_names[j] == "di.9") di9_index = j;
            }

            if (di1_index != -1 && di9_index != -1)
            {
                if (values.values[di1_index] == 0 || values.values[di9_index] == 0)
                {
                    new_emg = 0;
                }
            }
        }

        if (new_emg != emg_)
        {
            emg_ = new_emg;
            if (emg_ == 0)
            {
                RCLCPP_WARN(this->get_logger(), "EMG activated! di.1 or di.9 is LOW.");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "EMG deactivated! di.1 and di.9 are BOTH HIGH.");
                last_stopped_ = false;  // EMG devreden çıkınca hız sıfırsa tekrar durma moduna girebilsin
            }
        }
    }

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        bool new_charging = msg->power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        bool new_battery_full = msg->voltage >= 29.5;
    
        if (new_charging != charging_ || new_battery_full != battery_full_)
        {
            charging_ = new_charging;
            battery_full_ = new_battery_full && new_charging;
    
            if (battery_full_)
                RCLCPP_INFO(this->get_logger(), "Battery full! Voltage: %.2fV", msg->voltage);
            else if (charging_)
                RCLCPP_INFO(this->get_logger(), "Battery charging... Voltage: %.2fV", msg->voltage);
            else{
                RCLCPP_INFO(this->get_logger(), "Battery not charging and not full.");
                charging_= false;
                battery_full_ = false;
                }
            // Şarj durumu değiştiğinde ışıkların güncellenmesini sağlamak için timer_callback'i çağır
            // timer_callback();
        }
    }
    

    void timer_callback()
    {
        if ((this->now() - last_odom_time_).seconds() > 1.0)
        {
            last_twist_x_ = 0.0;
        }

        int new_direction = 0;
        if (last_twist_x_ < -0.01)
        {
            new_direction = -1;
            stop_counter_ = 0;
            last_stopped_ = false;
        }
        else if (last_twist_x_ > 0.01)
        {
            new_direction = 1;
            stop_counter_ = 0;
            last_stopped_ = false;
        }
        else 
        {
            stop_counter_++;
        }

        if (!charging_ && !battery_full_ && last_charging_)
        {
            RCLCPP_INFO(this->get_logger(), "Charging stopped, switching to stop mode.");
            new_direction = 0;  // Hareket yokmuş gibi algıla
            stop_counter_ = 10; // Direkt stop moduna girmesi için sayacı artır
            last_stopped_ = false;
        }
        
        if (new_direction == direction_ && emg_ == last_emg_ && charging_ == last_charging_ && battery_full_ == last_battery_full_ && ( last_stopped_ || stop_counter_ < 10))
        {
                return; // Gerçekten hiçbir değişiklik olmadıysa erken çık.
        }

        control_msgs::msg::DynamicInterfaceGroupValues control_msg;
        control_msg.header.stamp = this->get_clock()->now();
        control_msg.interface_groups = {"gpio0"};

        control_msgs::msg::InterfaceValue values_struct;
        values_struct.interface_names = {"do.1", "do.2", "do.3", "do.4", "do.9", "do.10", "do.11", "do.12"};

        if (battery_full_)
        {
            values_struct.values = {1, 0, 0, 0, 1, 0, 0, 0};
            RCLCPP_INFO(this->get_logger(), "Battery full! Special full-charge light pattern.");
        }
        else if (charging_)
        {
            values_struct.values = {1, 1, 0, 0, 1, 1, 0, 0};
            RCLCPP_INFO(this->get_logger(), "Charging mode active!");
        }
        else if (emg_ == 0)
        {
            values_struct.values = {0, 1, 1, 0, 0, 1, 1, 0};
            RCLCPP_WARN(this->get_logger(), "Emergency mode activated! Red lights FLASH");
        }
        else
        {
            if (new_direction == -1)
            {
                values_struct.values = {0, 1, 0, 0, 1, 0, 1, 0};
                RCLCPP_INFO(this->get_logger(), "Direction changed to -1");
            }
            else if (new_direction == 1)
            {
                values_struct.values = {1, 0, 1, 0, 0, 1, 0, 0};
                RCLCPP_INFO(this->get_logger(), "Direction changed to 1");
            }
            else
            {
                if (stop_counter_ >= 10 && !last_stopped_)
                {
                    values_struct.values = {1, 1, 1, 0, 1, 1, 1, 0};
                    RCLCPP_INFO(this->get_logger(), "Vehicle stopped for 1 second! Changing light mode.");
                    last_stopped_ = true;
                }
            }
        }

        if (!values_struct.values.empty()) 
        {
            control_msg.interface_values.push_back(values_struct);
            publisher_->publish(control_msg);
        } 

        direction_ = new_direction;
        last_emg_ = emg_;
        last_charging_ = charging_;
        last_battery_full_ = battery_full_;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr subscription_gpio_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_battery_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double last_twist_x_;
    int direction_, emg_, last_emg_, stop_counter_;
    bool charging_, battery_full_, last_charging_, last_battery_full_, last_stopped_;
    rclcpp::Time last_odom_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightControl>());
    rclcpp::shutdown();
    return 0;
}
