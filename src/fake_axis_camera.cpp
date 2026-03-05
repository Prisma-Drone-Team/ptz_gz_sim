#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ptz_action_server_msgs/action/ptz_move.hpp>
#include <ptz_action_server_msgs/msg/ptz.hpp>
#include <ptz_action_server_msgs/msg/ptz_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <thread>
#include <cmath>

class FakeAxisCamera : public rclcpp::Node
{
public:
    FakeAxisCamera();

private:
    // PTZ state variables
    float current_pan_;
    float current_tilt_;
    float current_zoom_;
    int8_t mode_;
    
    // ROS2 objects
    rclcpp::Publisher<ptz_action_server_msgs::msg::PtzState>::SharedPtr ptz_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ptz_zoom_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_cmd_pub_;
    rclcpp::Subscription<ptz_action_server_msgs::msg::Ptz>::SharedPtr cmd_velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ptz_zoom_fb_sub_;
    rclcpp_action::Server<ptz_action_server_msgs::action::PtzMove>::SharedPtr action_server_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_srv_;
    
    // Parameters
    const float dt = 1/20.0; 
    const float PAN_SPEED_ = 10.0f;      // degrees/command
    const float TILT_SPEED_ = 10.0f;     // degrees/command
    const float ZOOM_SPEED_ = 0.1f;      // per command
    const float PAN_MIN_ = -170.0f;
    const float PAN_MAX_ = 170.0f;
    const float TILT_MIN_ = -90.0f;
    const float TILT_MAX_ = 90.0f;
    const float ZOOM_MIN_ = 1.0f;
    const float ZOOM_MAX_ = 1.0f;  // Can adjust based on camera specs
    
    // Method declarations
    
    // callbacks
    void cmd_velocity_cb(const ptz_action_server_msgs::msg::Ptz::ConstSharedPtr msg);
    void joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
    void ptz_zoom_fb_cb(const std_msgs::msg::Float64::ConstSharedPtr msg);
    
    // server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ptz_action_server_msgs::action::PtzMove::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle);
    
    // utility functions
    float clamp(float value, float min_val, float max_val);
    void execute_move(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle);
};

// --- FakeAxisCamera method definitions ---

FakeAxisCamera::FakeAxisCamera() : Node("fake_axis_camera")
{
    // Initialize PTZ state
    current_pan_ = 0.0f;
    current_tilt_ = 0.0f;
    current_zoom_ = 1.0f;
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_IDLE;
    
    // Create publishers
    const rclcpp::QoS qos_best_effort = rclcpp::QoS(10).best_effort();
    const rclcpp::QoS qos_reliable = rclcpp::QoS(10).reliable();
    ptz_state_pub_ = this->create_publisher<ptz_action_server_msgs::msg::PtzState>("/ptz_state", qos_reliable);
    ptz_zoom_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ptz_zoom_cmd", qos_reliable);
    vel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", qos_reliable);
    pos_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", qos_reliable);
    
    // Create subscribers
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", qos_reliable, std::bind(&FakeAxisCamera::joint_state_cb, this, std::placeholders::_1));
    ptz_zoom_fb_sub_ = this->create_subscription<std_msgs::msg::Float64>("/ptz_zoom_fb", qos_reliable, std::bind(&FakeAxisCamera::ptz_zoom_fb_cb, this, std::placeholders::_1));
    cmd_velocity_sub_ = this->create_subscription<ptz_action_server_msgs::msg::Ptz>("/cmd/velocity", qos_reliable, std::bind(&FakeAxisCamera::cmd_velocity_cb, this, std::placeholders::_1));
    
    // Create action server for move_ptz/position_abs
    action_server_ = rclcpp_action::create_server<ptz_action_server_msgs::action::PtzMove>(
        this,
        "move_ptz/position_abs",
        std::bind(&FakeAxisCamera::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&FakeAxisCamera::handle_cancel, this, std::placeholders::_1),
        std::bind(&FakeAxisCamera::handle_accepted, this, std::placeholders::_1));

    // create client for switch controller service
    switch_controller_srv_ = this->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");
            
    RCLCPP_INFO(this->get_logger(), "FakeAxisCamera node started");
}

// Callback for /cmd/velocity topic
void FakeAxisCamera::cmd_velocity_cb(const ptz_action_server_msgs::msg::Ptz::ConstSharedPtr msg)
{
    // switch controller to velocity controller
    auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->activate_asap = true;
    switch_req->strictness = controller_manager_msgs::srv::SwitchController_Request::BEST_EFFORT;
    switch_req->activate_controllers = {"velocity_controller"};
    switch_req->deactivate_controllers = {"position_controller"};

    switch_controller_srv_->async_send_request(switch_req);

    // send velocity commands
    std_msgs::msg::Float64MultiArray send_msg_vel;
    
    // // Set mode to VELOCITY
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_VELOCITY;

    // // 
    // // Update position based on velocity command
    // current_pan_ = current_pan_ + msg->pan * dt;
    // current_tilt_ = current_tilt_ + msg->tilt * dt;
    // current_zoom_ = current_zoom_ + msg->zoom * dt;

    send_msg_vel.data.push_back(msg->pan);
    send_msg_vel.data.push_back(msg->tilt);

    vel_cmd_pub_->publish(send_msg_vel);

    std_msgs::msg::Float64 zoom_msg;
    zoom_msg.data = current_zoom_ + msg->zoom * dt;
    ptz_zoom_cmd_pub_->publish(zoom_msg);

    RCLCPP_INFO(this->get_logger(), "sent VEL msg: %f %f %f", send_msg_vel.data[0], send_msg_vel.data[1], zoom_msg.data);

}

// Publish current PTZ state
void FakeAxisCamera::joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    auto curr_pan = msg->position[0];
    auto curr_tilt = msg->position[1];
    auto curr_zoom = current_zoom_;
    
    auto send_msg = ptz_action_server_msgs::msg::PtzState();
    send_msg.mode = mode_;
    send_msg.pan = curr_pan;
    send_msg.tilt = curr_tilt;
    send_msg.zoom = curr_zoom;
    ptz_state_pub_->publish(send_msg);
}

void FakeAxisCamera::ptz_zoom_fb_cb(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    current_zoom_ = msg->data;
}

// Action server callbacks
rclcpp_action::GoalResponse FakeAxisCamera::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ptz_action_server_msgs::action::PtzMove::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), 
        "Received move goal: pan=%.2f, tilt=%.2f, zoom=%.2f",
        goal->ptz.pan, goal->ptz.tilt, goal->ptz.zoom);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FakeAxisCamera::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FakeAxisCamera::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle)
{
    // switch controller to position controller
    auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->activate_asap = true;
    switch_req->strictness = controller_manager_msgs::srv::SwitchController_Request::BEST_EFFORT;
    switch_req->activate_controllers = {"position_controller"};
    switch_req->deactivate_controllers = {"velocity_controller"};

    switch_controller_srv_->async_send_request(switch_req);

    // send position commands
    std_msgs::msg::Float64MultiArray send_msg_pos;
    
    // // Set mode to POSITION
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_POSITION;
    
    send_msg_pos.data.push_back(goal_handle->get_goal()->ptz.pan);
    send_msg_pos.data.push_back(goal_handle->get_goal()->ptz.tilt);

    pos_cmd_pub_->publish(send_msg_pos);

    std_msgs::msg::Float64 zoom_msg;
    zoom_msg.data = goal_handle->get_goal()->ptz.zoom;
    ptz_zoom_cmd_pub_->publish(zoom_msg);

    RCLCPP_INFO(this->get_logger(), "sent POS msg: %f %f %f", send_msg_pos.data[0], send_msg_pos.data[1], zoom_msg.data);

    auto res_msg = std::make_shared<ptz_action_server_msgs::action::PtzMove_Result>();
    res_msg->success = true;
    res_msg->message = "GOAL!!!";
    goal_handle->succeed(res_msg);

    // Execute action in separate thread
    //std::thread(&FakeAxisCamera::execute_move, this, goal_handle).detach();
}

// Clamp value between min and max
float FakeAxisCamera::clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

void FakeAxisCamera::execute_move(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle)
{
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ptz_action_server_msgs::action::PtzMove::Feedback>();
    
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_POSITION;
    
    // Simulate gradual movement to target position
    const float STEP_SIZE = 1.0f;  // Movement step per iteration
    const float ZOOM_STEP = 0.01f;
    const int SLEEP_MS = 50;       // 50ms between steps
    
    float target_pan = goal->ptz.pan;
    float target_tilt = goal->ptz.tilt;
    float target_zoom = goal->ptz.zoom;
    
    RCLCPP_INFO(this->get_logger(),
        "Starting move to: pan=%.2f, tilt=%.2f, zoom=%.2f",
        target_pan, target_tilt, target_zoom);
    
    bool reached_target = false;
    
    while (rclcpp::ok() && !reached_target)
    {
        if (goal_handle->is_canceling())
        {
            RCLCPP_INFO(this->get_logger(), "Move action cancelled");
            auto result = std::make_shared<ptz_action_server_msgs::action::PtzMove::Result>();
            result->success = false;
            result->message = "Move action cancelled";
            goal_handle->canceled(result);
            mode_ = ptz_action_server_msgs::msg::PtzState::MODE_IDLE;
            return;
        }
        
        // Calculate distance to target
        float pan_diff = target_pan - current_pan_;
        float tilt_diff = target_tilt - current_tilt_;
        float zoom_diff = target_zoom - current_zoom_;
        
        // Move one step closer to target
        if (std::abs(pan_diff) > STEP_SIZE)
        {
            current_pan_ += (pan_diff > 0) ? STEP_SIZE : -STEP_SIZE;
        }
        else if (std::abs(pan_diff) > 0.01f)
        {
            current_pan_ = target_pan;
        }
        
        if (std::abs(tilt_diff) > STEP_SIZE)
        {
            current_tilt_ += (tilt_diff > 0) ? STEP_SIZE : -STEP_SIZE;
        }
        else if (std::abs(tilt_diff) > 0.01f)
        {
            current_tilt_ = target_tilt;
        }
        
        if (std::abs(zoom_diff) > ZOOM_STEP)
        {
            current_zoom_ += (zoom_diff > 0) ? ZOOM_STEP : -ZOOM_STEP;
        }
        else if (std::abs(zoom_diff) > 0.001f)
        {
            current_zoom_ = target_zoom;
        }
        
        // Check if we reached target
        if (std::abs(pan_diff) <= 0.01f &&
            std::abs(tilt_diff) <= 0.01f &&
            std::abs(zoom_diff) <= 0.001f)
        {
            reached_target = true;
        }
        
        // Send feedback
        feedback->ptz_remaining.pan = target_pan - current_pan_;
        feedback->ptz_remaining.tilt = target_tilt - current_tilt_;
        feedback->ptz_remaining.zoom = target_zoom - current_zoom_;
        goal_handle->publish_feedback(feedback);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    }
    
    // Action completed successfully
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_IDLE;
    
    auto result = std::make_shared<ptz_action_server_msgs::action::PtzMove::Result>();
    result->success = true;
    result->message = "Successfully moved to target position";
    
    RCLCPP_INFO(this->get_logger(),
        "Move completed: pan=%.2f, tilt=%.2f, zoom=%.2f",
        current_pan_, current_tilt_, current_zoom_);
    
    goal_handle->succeed(result);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FakeAxisCamera>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
