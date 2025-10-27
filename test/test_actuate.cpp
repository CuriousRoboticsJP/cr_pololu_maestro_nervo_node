#include "cr_hand_interfaces/msg/hand_state.hpp"
#include "cr_hand_interfaces/msg/hand_command.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class TestMaestroNode : public rclcpp::Node
{
public:
    TestMaestroNode(const rclcpp::NodeOptions &options) : Node("Test_CR_Servo_Node", options)
    {
        this->last_command_ = TestMaestroNode::CommandState::Open;

        // initialize servo data
        this->servoCommand = new double[TestMaestroNode::num_servos]();
        std::fill(this->servoCommand, this->servoCommand + TestMaestroNode::num_servos, 0.0);
        this->servoState = new double[TestMaestroNode::num_servos]();
        std::fill(this->servoState, this->servoState + TestMaestroNode::num_servos, 0.0);

        // create a reentrant callback group
        auto reentrant_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = reentrant_group;

        // hand state subscriber
        this->sub_hand_state = this->create_subscription<cr_hand_interfaces::msg::HandState>("/hand_state", 10,
            [this](const cr_hand_interfaces::msg::HandState::SharedPtr msg)
            {
                this->servoState[0] = msg->palm.position[0];
                this->servoState[1] = msg->thumb.position[0];
                this->servoState[2] = msg->index.position[0];
                this->servoState[3] = msg->middle.position[0];
                this->servoState[4] = msg->ring.position[0];
                this->servoState[5] = msg->little.position[0];
                this->updateState();
            }, sub_options);

        // hand command publisher
        this->pub_hand_command = this->create_publisher<cr_hand_interfaces::msg::HandCommand>("/hand_command", 10);
        this->exec_state_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(this->time_pub_command_),
            [this]()
            {
                this->sendCommand();
            });
    }

    ~TestMaestroNode()
    {
        // explicitly destroy ros objects
        this->pub_hand_command.reset();
        this->exec_state_timer_.reset();
        this->sub_hand_state.reset();
    }

    enum class CommandState
    {
        Open,
        PinchOpen,
        PinchClose,
        GraspOpen,
        GraspClose,
        KeyPinchOpen,
        KeyPinchClose,
    };

    struct Open
    {
        constexpr static double palm = 0.0;
        constexpr static double thumb = 0.0;
        constexpr static double index = 0.0;
        constexpr static double middle = 0.0;
        constexpr static double ring = 0.0;
        constexpr static double little = 0.0;
    };

    struct PinchOpen
    {
        constexpr static double palm = 1.0;
        constexpr static double thumb = 0.752;
        constexpr static double index = 0.0;
        constexpr static double middle = 1.0;
        constexpr static double ring = 1.0;
        constexpr static double little = 1.0;
    };

    struct PinchClose
    {
        constexpr static double palm = 1.0;
        constexpr static double thumb = 0.752;
        constexpr static double index = 0.504;
        constexpr static double middle = 1.0;
        constexpr static double ring = 1.0;
        constexpr static double little = 1.0;
    };

    struct GraspOpen
    {
        constexpr static double palm = 1.0;
        constexpr static double thumb = 0.0;
        constexpr static double index = 0.0;
        constexpr static double middle = 0.0;
        constexpr static double ring = 0.0;
        constexpr static double little = 0.0;
    };

    struct GraspClose
    {
        constexpr static double palm = 1.0;
        constexpr static double thumb = 0.504;
        constexpr static double index = 1.0;
        constexpr static double middle = 1.0;
        constexpr static double ring = 1.0;
        constexpr static double little = 1.0;
    };

    struct KeyPinchOpen
    {
        constexpr static double palm = 0.0;
        constexpr static double thumb = 0.0;
        constexpr static double index = 1.0;
        constexpr static double middle = 1.0;
        constexpr static double ring = 1.0;
        constexpr static double little = 1.0;
    };

    struct KeyPinchClose
    {
        constexpr static double palm = 0.0;
        constexpr static double thumb = 1.0;
        constexpr static double index = 1.0;
        constexpr static double middle = 1.0;
        constexpr static double ring = 1.0;
        constexpr static double little = 1.0;
    };

private:
    int time_pub_command_ = 1250;
    rclcpp::TimerBase::SharedPtr exec_state_timer_;
    rclcpp::Publisher<cr_hand_interfaces::msg::HandCommand>::SharedPtr pub_hand_command;
    rclcpp::Subscription<cr_hand_interfaces::msg::HandState>::SharedPtr sub_hand_state;

    TestMaestroNode::CommandState last_command_;
    double *servoCommand;
    double *servoState;

    void sendCommand()
    {
        auto msg_HandCommand = cr_hand_interfaces::msg::HandCommand();
        switch(this->last_command_)
        {
            case TestMaestroNode::CommandState::Open:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::PinchOpen::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::PinchOpen::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::PinchOpen::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::PinchOpen::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::PinchOpen::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::PinchOpen::little;
                break;
            case TestMaestroNode::CommandState::PinchOpen:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::PinchClose::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::PinchClose::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::PinchClose::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::PinchClose::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::PinchClose::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::PinchClose::little;
                break;
            case TestMaestroNode::CommandState::PinchClose:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::GraspOpen::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::GraspOpen::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::GraspOpen::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::GraspOpen::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::GraspOpen::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::GraspOpen::little;
                break;
            case TestMaestroNode::CommandState::GraspOpen:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::GraspClose::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::GraspClose::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::GraspClose::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::GraspClose::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::GraspClose::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::GraspClose::little;
                break;
            case TestMaestroNode::CommandState::GraspClose:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::KeyPinchOpen::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::KeyPinchOpen::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::KeyPinchOpen::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::KeyPinchOpen::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::KeyPinchOpen::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::KeyPinchOpen::little;
                break;
            case TestMaestroNode::CommandState::KeyPinchOpen:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::KeyPinchClose::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::KeyPinchClose::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::KeyPinchClose::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::KeyPinchClose::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::KeyPinchClose::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::KeyPinchClose::little;
                break;
            case TestMaestroNode::CommandState::KeyPinchClose:
                msg_HandCommand.palm_pos_cmd = TestMaestroNode::Open::palm;
                msg_HandCommand.thumb_pos_cmd = TestMaestroNode::Open::thumb;
                msg_HandCommand.index_pos_cmd = TestMaestroNode::Open::index;
                msg_HandCommand.middle_pos_cmd = TestMaestroNode::Open::middle;
                msg_HandCommand.ring_pos_cmd = TestMaestroNode::Open::ring;
                msg_HandCommand.little_pos_cmd = TestMaestroNode::Open::little;
                break;
        }
        this->pub_hand_command->publish(msg_HandCommand);
    }

    void updateState()
    {
        if (this->compare(this->servoState, TestMaestroNode::CommandState::Open))
        {
            this->last_command_ = TestMaestroNode::CommandState::Open;
        }
        else if (this->compare(this->servoState, TestMaestroNode::CommandState::PinchOpen))
        {
            this->last_command_ = TestMaestroNode::CommandState::PinchOpen;
        }
        else if (this->compare(this->servoState, TestMaestroNode::CommandState::PinchClose))
        {
            this->last_command_ = TestMaestroNode::CommandState::PinchClose;
        }
        else if (this->compare(this->servoState, TestMaestroNode::CommandState::GraspOpen))
        {
            this->last_command_ = TestMaestroNode::CommandState::GraspOpen;
        }
        else if (this->compare(this->servoState, TestMaestroNode::CommandState::GraspClose))
        {
            this->last_command_ = TestMaestroNode::CommandState::GraspClose;
        }
        else if (this->compare(this->servoState, TestMaestroNode::CommandState::KeyPinchOpen))
        {
            this->last_command_ = TestMaestroNode::CommandState::KeyPinchOpen;
        }
        else if (this->compare(this->servoState, TestMaestroNode::CommandState::KeyPinchClose))
        {
            this->last_command_ = TestMaestroNode::CommandState::KeyPinchClose;
        }
    }

    bool compare(double *state, TestMaestroNode::CommandState commandState)
    {
        bool ret_val = false;
        switch (commandState)
        {
        case TestMaestroNode::CommandState::Open:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::Open::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::Open::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::Open::index) &&
                       this->compareDouble(state[3], TestMaestroNode::Open::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::Open::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::Open::little));
            break;
        case TestMaestroNode::CommandState::PinchOpen:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::PinchOpen::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::PinchOpen::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::PinchOpen::index) &&
                       this->compareDouble(state[3], TestMaestroNode::PinchOpen::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::PinchOpen::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::PinchOpen::little));
            break;
        case TestMaestroNode::CommandState::PinchClose:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::PinchClose::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::PinchClose::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::PinchClose::index) &&
                       this->compareDouble(state[3], TestMaestroNode::PinchClose::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::PinchClose::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::PinchClose::little));
            break;
        case TestMaestroNode::CommandState::GraspOpen:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::GraspOpen::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::GraspOpen::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::GraspOpen::index) &&
                       this->compareDouble(state[3], TestMaestroNode::GraspOpen::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::GraspOpen::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::GraspOpen::little));
            break;
        case TestMaestroNode::CommandState::GraspClose:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::GraspClose::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::GraspClose::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::GraspClose::index) &&
                       this->compareDouble(state[3], TestMaestroNode::GraspClose::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::GraspClose::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::GraspClose::little));
            break;
        case TestMaestroNode::CommandState::KeyPinchOpen:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::KeyPinchOpen::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::KeyPinchOpen::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::KeyPinchOpen::index) &&
                       this->compareDouble(state[3], TestMaestroNode::KeyPinchOpen::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::KeyPinchOpen::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::KeyPinchOpen::little));
            break;
        case TestMaestroNode::CommandState::KeyPinchClose:
            ret_val = (this->compareDouble(state[0], TestMaestroNode::KeyPinchClose::palm) &&
                       this->compareDouble(state[1], TestMaestroNode::KeyPinchClose::thumb) &&
                       this->compareDouble(state[2], TestMaestroNode::KeyPinchClose::index) &&
                       this->compareDouble(state[3], TestMaestroNode::KeyPinchClose::middle) &&
                       this->compareDouble(state[4], TestMaestroNode::KeyPinchClose::ring) &&
                       this->compareDouble(state[5], TestMaestroNode::KeyPinchClose::little));
            break;
        }
        return ret_val;
    }

    bool compareDouble(double a, double b)
    {
        return (std::fabs(a - b) <= TestMaestroNode::epsilon ? true : false);
    }

protected:
    constexpr static int num_servos = 6;
    constexpr static double epsilon = 0.001;

};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestMaestroNode)