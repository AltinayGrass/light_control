#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "control_msgs/msg/dynamic_interface_group_values.hpp"
#include "control_msgs/msg/interface_value.hpp"

class LightControl : public rclcpp::Node
{
public:
    LightControl() : Node("light_control")
    {
        // Odom Subscriber
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/diff_drive_controller/odom", 10, 
            std::bind(&LightControl::odom_callback, this, std::placeholders::_1));

        // GPIO States Subscriber
        subscription_gpio_ = this->create_subscription<control_msgs::msg::DynamicInterfaceGroupValues>(
            "/gpio_controller/gpio_states", 10, 
            std::bind(&LightControl::gpio_callback, this, std::placeholders::_1));

        // Publisher
        publisher_ = this->create_publisher<control_msgs::msg::DynamicInterfaceGroupValues>(
            "/gpio_controller/commands", 10);

        last_twist_x_ = 0.0;
        direction_ = 0;
        emg_ = 1;  // Başlangıçta EMG aktif
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double twist_x = msg->twist.twist.linear.x;

        // Yön değişimini belirle
        int new_direction = 0;
        if (twist_x < -0.05) {
            new_direction = -1;
        } else if (twist_x > 0.05) {
            new_direction = 1;
        }

        // Eğer yön değişmemişse ve emg değişmemişse, yayın yapma
        if (new_direction == direction_ && emg_ == last_emg_) {
            return;
        }

        // Yeni mesaj nesnesi oluştur
        auto control_msg = control_msgs::msg::DynamicInterfaceGroupValues();
        control_msg.header.stamp = this->get_clock()->now();
        control_msg.header.frame_id = "light_control";
        control_msg.interface_groups = {"gpio0"};

        control_msgs::msg::InterfaceValue values_struct;
        values_struct.interface_names = {"do.1", "do.2", "do.3", "do.4", 
                                         "do.9", "do.10", "do.11", "do.12"};

        // Eğer acil durum aktifse, tüm ışıkları kapat
        if (emg_ == 1) {
            values_struct.values = {0, 1, 1, 0, 0, 1, 1, 0};  // Kirmizi yan son
            RCLCPP_WARN(this->get_logger(), "Emergency mode activated! Red lights FLASH");
        }
        else {
            // Yeni yön bilgisine göre ışık düzenini belirle
            if (new_direction == -1) {
                values_struct.values = {0, 1, 0, 0, 1, 0, 1, 0};
                RCLCPP_INFO(this->get_logger(), "Direction changed to -1");
            } 
            else if (new_direction == 1) {
                values_struct.values = {1, 0, 1, 0, 0, 1, 0, 0};
                RCLCPP_INFO(this->get_logger(), "Direction changed to 1");
            } 
            else { // new_direction == 0
                values_struct.values = {1, 1, 1, 0, 1, 1, 1, 0};
                RCLCPP_INFO(this->get_logger(), "Direction changed to 0");
            }
        }

        control_msg.interface_values.push_back(values_struct);

        // Mesajı yayınla
        publisher_->publish(control_msg);

        // Son değerleri güncelle
        direction_ = new_direction;
        last_twist_x_ = twist_x;
        last_emg_ = emg_;
    }

    void gpio_callback(const control_msgs::msg::DynamicInterfaceGroupValues::SharedPtr msg)
    {
        int new_emg = 1;  // Varsayılan olarak EMG aktif
    
        // GPIO1 grubunu ara
        for (size_t i = 0; i < msg->interface_groups.size(); i++)
        {
            if (msg->interface_groups[i] == "gpio1") // GPIO1 grubunu bulduk
            {
                for (const auto &values : msg->interface_values)
                {
                    int di1_index = -1, di9_index = -1;
                    for (size_t j = 0; j < values.interface_names.size(); j++)
                    {
                        if (values.interface_names[j] == "di.1") {
                            di1_index = j;
                        }
                        if (values.interface_names[j] == "di.9") {
                            di9_index = j;
                        }
                    }
    
                    if (di1_index == -1 || di9_index == -1) {
                        continue;
                    }
    
                    // Eğer ikisi de 1 ise EMG = 0, diğer tüm durumlarda EMG = 1
                    if (values.values[di1_index] == 1 && values.values[di9_index] == 1) {
                        new_emg = 0;
                    }
    
                    break; // İlgili grup bulunduğunda döngüden çık
                }
            }
        }
    
        // **Sadece değişiklik olduğunda güncelle ve log yazdır**
        if (new_emg != emg_) {
            emg_ = new_emg;
    
            if (emg_ == 0) {
                RCLCPP_INFO(this->get_logger(), "EMG deactivated! di.1 and di.9 are BOTH HIGH.");
            } else {
                RCLCPP_WARN(this->get_logger(), "EMG activated! di.1 or di.9 is LOW.");
            }
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::Subscription<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr subscription_gpio_;
    rclcpp::Publisher<control_msgs::msg::DynamicInterfaceGroupValues>::SharedPtr publisher_;
    
    double last_twist_x_;
    int direction_;
    int emg_;
    int last_emg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightControl>());
    rclcpp::shutdown();
    return 0;
}
