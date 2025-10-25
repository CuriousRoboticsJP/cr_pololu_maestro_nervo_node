#include "usb_device.h"
#include "maestro_device.h"

#include "cr_hand_interfaces/msg/hand_state.hpp"
#include "cr_hand_interfaces/msg/hand_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <stdexcept>

using namespace std::chrono_literals;

class MaestroNode : public rclcpp::Node
{
public:
    MaestroNode(const rclcpp::NodeOptions &options) : Node("CR_Servo_Node", options)
    {
        this->maestroDevice = MaestroDevice::Open(MaestroDevice::ProductID::Maestro_6ch);
        if (!this->maestroDevice)
        {
            RCLCPP_ERROR(get_logger(), "Failed to open Pololu Maestro USB device!");
            throw std::runtime_error("Failed to open Pololu Maestro USB device");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Pololu Maestro device opened!");
        }

        // initialize servo data
        this->servoCommand = new double[this->maestroDevice->GetServoCount()]();
        std::fill(this->servoCommand, this->servoCommand + this->maestroDevice->GetServoCount(), 0.0);
        this->servoState = new double[this->maestroDevice->GetServoCount()]();
        std::fill(this->servoState, this->servoState + this->maestroDevice->GetServoCount(), 0.0);
        this->WriteCommand();

        // create a reentrant callback group
        auto reentrant_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = reentrant_group;

        // hand command subscriber
        this->sub_hand_command = this->create_subscription<cr_hand_interfaces::msg::HandCommand>("/hand_command", 10,
            [this](const cr_hand_interfaces::msg::HandCommand::SharedPtr msg)
            {
                this->rx_.try_lock();
                this->servoCommand[0] = msg->palm_pos_cmd;
                this->servoCommand[1] = msg->thumb_pos_cmd;
                this->servoCommand[2] = msg->index_pos_cmd;
                this->servoCommand[3] = msg->middle_pos_cmd;
                this->servoCommand[4] = msg->ring_pos_cmd;
                this->servoCommand[5] = msg->little_pos_cmd;
                this->rx_.unlock();
            }, sub_options);

        // hand state publisher
        this->pub_hand_state = this->create_publisher<cr_hand_interfaces::msg::HandState>("/hand_state", 10);
        this->exec_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(this->time_pub_state_),
            [this]()
            {
                // this->ReadState();
                this->pubState();
                this->WriteCommand();
            });
    }

    ~MaestroNode()
    {
        // explicitly destroy ros objects
        this->sub_hand_command.reset();
        this->exec_state_timer_.reset();
        this->pub_hand_state.reset();

        // destroy the maestro device handle
        if (this->maestroDevice != NULL)
        {
            delete this->maestroDevice;
            this->maestroDevice = NULL;
        }
    }

private:
    int time_pub_state_ = 20;
    rclcpp::TimerBase::SharedPtr exec_state_timer_;
    rclcpp::Publisher<cr_hand_interfaces::msg::HandState>::SharedPtr pub_hand_state;
    rclcpp::Subscription<cr_hand_interfaces::msg::HandCommand>::SharedPtr sub_hand_command;

    MaestroDevice *maestroDevice;
    double *servoCommand;
    double *servoState;
    std::mutex rx_;
    std::mutex tx_;

    void pubState()
    {
        auto msg_HandState = cr_hand_interfaces::msg::HandState();
        msg_HandState.palm = sensor_msgs::msg::JointState();
        msg_HandState.thumb = sensor_msgs::msg::JointState();
        msg_HandState.index = sensor_msgs::msg::JointState();
        msg_HandState.middle = sensor_msgs::msg::JointState();
        msg_HandState.ring = sensor_msgs::msg::JointState();
        msg_HandState.little = sensor_msgs::msg::JointState();

        this->tx_.try_lock();
        msg_HandState.palm.name.push_back("palm");
        msg_HandState.palm.position.push_back(this->servoState[0]);

        msg_HandState.thumb.name.push_back("thumb");
        msg_HandState.thumb.position.push_back(this->servoState[1]);

        msg_HandState.index.name.push_back("index");
        msg_HandState.index.position.push_back(this->servoState[2]);

        msg_HandState.middle.name.push_back("middle");
        msg_HandState.middle.position.push_back(this->servoState[3]);

        msg_HandState.ring.name.push_back("ring");
        msg_HandState.ring.position.push_back(this->servoState[4]);

        msg_HandState.little.name.push_back("little");
        msg_HandState.little.position.push_back(this->servoState[5]);
        this->tx_.unlock();

        this->pub_hand_state->publish(msg_HandState);
    }

    void WriteCommand()
    {
        this->rx_.try_lock();
        this->tx_.try_lock();
        for (auto i = 0; i < this->maestroDevice->GetServoCount(); i++)
        {
            this->maestroDevice->SetPosition(i, this->maestroDevice->ConvertToPWMPosition(this->servoCommand[i]));
            this->servoState[i] = this->servoCommand[i];
        }
        this->tx_.unlock();
        this->rx_.unlock();
    }
};

int main(int argc, char *argv[])
{
    try
    {
        // Initialize ROS
        rclcpp::init(argc, argv);

        // create the node
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        auto node = std::make_shared<MaestroNode>(node_options);
        RCLCPP_INFO(node->get_logger(), "Application Started!");

        // create multi-threaded executor
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        while (rclcpp::ok())
        {
            executor.spin_once();
        }

        // Shutdown ROS
        RCLCPP_INFO(node->get_logger(), "Application Stopped!");
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cout << "__ERROR: " << e.what();
    }
    catch (...)
    {
        std::cout << "__ERROR: Unknown Exception!";
    }

    return 0;
}
