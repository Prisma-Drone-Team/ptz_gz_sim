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
#include <chrono>

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
    std::string current_active_controller_;
    
    // ROS2 objects
    rclcpp::Publisher<ptz_action_server_msgs::msg::PtzState>::SharedPtr ptz_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ptz_zoom_cmd_pub_;   // NEW: Real zoom command publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ptz_zoom_fb_pub_;    // Zoom feedback publisher  
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pos_cmd_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;  // NEW: Publish for RViz
    rclcpp::Subscription<ptz_action_server_msgs::msg::Ptz>::SharedPtr cmd_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ptz_zoom_cmd_sub_;  // Zoom command subscriber
    rclcpp_action::Server<ptz_action_server_msgs::action::PtzMove>::SharedPtr action_server_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_srv_;
    
    // Gazebo publishers for direct joint control via topics
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pan_joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tilt_joint_pub_;
    
    // Timer for velocity publishing
    rclcpp::TimerBase::SharedPtr velocity_timer_;
    
    // Current velocity commands 
    float current_vel_pan_ = 0.0f;
    float current_vel_tilt_ = 0.0f;
    float current_vel_zoom_ = 0.0f;
    
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
    const float ZOOM_MAX_ = 125.0f; // Full zoom range for axis camera (restored original value)
    
    // Method declarations
    
    // callbacks
    void cmd_velocity_cb(const ptz_action_server_msgs::msg::Ptz::ConstSharedPtr msg);
    void ptz_zoom_cmd_cb(const std_msgs::msg::Float64::ConstSharedPtr msg);  // NEW: Direct zoom command callback
    
    // server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ptz_action_server_msgs::action::PtzMove::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle);
    
    // utility functions
    float clamp(float value, float min_val, float max_val);
    void execute_move(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ptz_action_server_msgs::action::PtzMove>> goal_handle);
    bool switch_to_controller(const std::string& target_controller);
    void velocity_timer_callback();
    void force_gazebo_joint_positions(float pan_rad, float tilt_rad);
    void publish_joint_states();  // NEW: Publish joint states for RViz
};

// --- FakeAxisCamera method definitions ---

FakeAxisCamera::FakeAxisCamera() : Node("fake_axis_camera")
{
    // Initialize PTZ state
    current_pan_ = 0.0f;
    current_tilt_ = 0.0f;
    current_zoom_ = 1.0f;
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_IDLE;
    current_active_controller_ = "position_controller"; // Default to position controller
    
    // Create publishers
    const rclcpp::QoS qos_reliable = rclcpp::QoS(10).reliable();
    ptz_state_pub_ = this->create_publisher<ptz_action_server_msgs::msg::PtzState>("/ptz_state", qos_reliable);
    ptz_zoom_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ptz_zoom_cmd", qos_reliable);   // NEW: Real zoom command
    ptz_zoom_fb_pub_ = this->create_publisher<std_msgs::msg::Float64>("/ptz_zoom_fb", qos_reliable);  // NEW: Zoom feedback
    vel_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", qos_reliable);
    pos_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", qos_reliable);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos_reliable);  // NEW: For RViz
    
    // Create subscribers
    ptz_zoom_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>("/ptz_zoom_cmd", qos_reliable, std::bind(&FakeAxisCamera::ptz_zoom_cmd_cb, this, std::placeholders::_1));  // NEW: Zoom commands
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
    
    // Create Gazebo joint position publishers (working config from minimal_sim)
    pan_joint_pub_ = this->create_publisher<std_msgs::msg::Float64>("/pan_cmd", qos_reliable);
    tilt_joint_pub_ = this->create_publisher<std_msgs::msg::Float64>("/tilt_cmd", qos_reliable);
    
    // Create timer for velocity publishing (50Hz) 
    velocity_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&FakeAxisCamera::velocity_timer_callback, this));
    
    // Initialize velocity commands
    current_vel_pan_ = 0.0f;
    current_vel_tilt_ = 0.0f;
    current_vel_zoom_ = 0.0f;
    
    RCLCPP_INFO(this->get_logger(), "FakeAxisCamera node started with working Gazebo Garden plugin config");
}

// Callback for /cmd/velocity topic - real velocity control!
void FakeAxisCamera::cmd_velocity_cb(const ptz_action_server_msgs::msg::Ptz::ConstSharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received velocity command: pan=%f, tilt=%f, zoom=%f", msg->pan, msg->tilt, msg->zoom);
    
    // Store velocity commands
    current_vel_pan_ = msg->pan;
    current_vel_tilt_ = msg->tilt;
    current_vel_zoom_ = msg->zoom;

    // Set mode to VELOCITY
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_VELOCITY;
    
    // HYBRID STRATEGY: Keep position_controller active for real movement
    // velocity_controller is only used for joint_states feedback
    if (current_active_controller_ != "position_controller") {
        RCLCPP_INFO(this->get_logger(), "Switching to position_controller for velocity integration...");
        switch_to_controller("position_controller");
    } else {
        RCLCPP_INFO(this->get_logger(), "Using position_controller for velocity integration");
    }
    
    RCLCPP_INFO(this->get_logger(), "Velocity mode activated: pan_vel=%f, tilt_vel=%f", current_vel_pan_, current_vel_tilt_);
}

// Publish current PTZ state
// Joint state callback replaced with publish function (no more ROS2 Control dependency)
void FakeAxisCamera::publish_joint_states()
{
    // Publish joint states for RViz visualization
    auto joint_msg = sensor_msgs::msg::JointState();
    joint_msg.header.stamp = this->now();
    joint_msg.name = {"pan_joint", "tilt_joint"};
    joint_msg.position = {current_pan_, current_tilt_};
    joint_msg.velocity = {current_vel_pan_, current_vel_tilt_};
    joint_state_pub_->publish(joint_msg);
    
    // Publish PTZ state
    auto ptz_msg = ptz_action_server_msgs::msg::PtzState();
    ptz_msg.mode = mode_;
    ptz_msg.pan = current_pan_;
    ptz_msg.tilt = current_tilt_;
    ptz_msg.zoom = current_zoom_;
    ptz_state_pub_->publish(ptz_msg);
    
    // Publish zoom feedback (pezzotto zoom implementation)
    auto zoom_fb_msg = std_msgs::msg::Float64();
    zoom_fb_msg.data = current_zoom_;
    ptz_zoom_fb_pub_->publish(zoom_fb_msg);
    
    // Force joint positions in Gazebo
    force_gazebo_joint_positions(current_pan_, current_tilt_);
    
    // Debug output for velocity mode
    static int counter = 0;
    counter++;
    if (mode_ == ptz_action_server_msgs::msg::PtzState::MODE_VELOCITY && counter % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), 
            "PTZ State: mode=VELOCITY, pan=%f, tilt=%f, zoom=%f", 
            current_pan_, current_tilt_, current_zoom_);
    }
}

// NEW: Direct zoom command callback (PEZZOTTO implementation)
void FakeAxisCamera::ptz_zoom_cmd_cb(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    float target_zoom = clamp(msg->data, ZOOM_MIN_, ZOOM_MAX_);
    current_zoom_ = target_zoom;
    
    RCLCPP_INFO(this->get_logger(), 
        "🎯 PEZZOTTO ZOOM: Received direct zoom command %.2f -> set to %.2f", 
        msg->data, current_zoom_);
    
    // Zoom feedback is published automatically in joint_state_cb via ptz_zoom_fb_pub_
}

// Removed: ptz_zoom_fb_cb (not needed in pezzotto implementation)

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
    RCLCPP_INFO(this->get_logger(), "Received position command: pan=%f, tilt=%f", 
                goal_handle->get_goal()->ptz.pan, goal_handle->get_goal()->ptz.tilt);
    
    // Switch to position controller if needed
    if (!switch_to_controller("position_controller")) {
        RCLCPP_ERROR(this->get_logger(), "Failed to switch to position controller");
        auto res_msg = std::make_shared<ptz_action_server_msgs::action::PtzMove_Result>();
        res_msg->success = false;
        res_msg->message = "Failed to switch to position controller";
        goal_handle->abort(res_msg);
        return;
    }

    // Set mode to POSITION and stop velocity
    mode_ = ptz_action_server_msgs::msg::PtzState::MODE_POSITION;
    current_vel_pan_ = 0.0f;
    current_vel_tilt_ = 0.0f;
    current_vel_zoom_ = 0.0f;
    
    // Send position commands
    std_msgs::msg::Float64MultiArray send_msg_pos;
    send_msg_pos.data.push_back(goal_handle->get_goal()->ptz.pan);
    send_msg_pos.data.push_back(goal_handle->get_goal()->ptz.tilt);
    pos_cmd_pub_->publish(send_msg_pos);

    // Send zoom command to real CameraZoomPlugin
    current_zoom_ = clamp(goal_handle->get_goal()->ptz.zoom, ZOOM_MIN_, ZOOM_MAX_);
    std_msgs::msg::Float64 zoom_msg;
    zoom_msg.data = current_zoom_;
    ptz_zoom_cmd_pub_->publish(zoom_msg);
    
    RCLCPP_INFO(this->get_logger(), "🔍 REAL ZOOM: Action server sent zoom=%.2f to CameraZoomPlugin", current_zoom_);

    RCLCPP_INFO(this->get_logger(), "Sent POS command: pan=%f, tilt=%f", send_msg_pos.data[0], send_msg_pos.data[1]);

    auto res_msg = std::make_shared<ptz_action_server_msgs::action::PtzMove_Result>();
    res_msg->success = true;
    res_msg->message = "Position command sent successfully";
    goal_handle->succeed(res_msg);
}

// Clamp value between min and max
float FakeAxisCamera::clamp(float value, float min_val, float max_val)
{
    return std::max(min_val, std::min(max_val, value));
}

// Ultra-simplified controller switch - pure fire and forget!
bool FakeAxisCamera::switch_to_controller(const std::string& target_controller)
{
    if (current_active_controller_ == target_controller) {
        RCLCPP_DEBUG(this->get_logger(), "Controller %s already active", target_controller.c_str());
        return true;
    }

    RCLCPP_INFO(this->get_logger(), "Switching from %s to %s...", 
               current_active_controller_.c_str(), target_controller.c_str());
    
    // Prepare switch request
    auto switch_req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->activate_asap = true;
    switch_req->strictness = controller_manager_msgs::srv::SwitchController_Request::BEST_EFFORT;
    
    // Always deactivate the other controller
    std::string other_controller = (target_controller == "position_controller") ? "velocity_controller" : "position_controller";
    
    switch_req->activate_controllers.push_back(target_controller);
    switch_req->deactivate_controllers.push_back(other_controller);
    
    // Fire and forget approach - no waiting at all!
    if (switch_controller_srv_->service_is_ready()) {
        switch_controller_srv_->async_send_request(switch_req);
        current_active_controller_ = target_controller;
        RCLCPP_INFO(this->get_logger(), "Requested switch to %s (pure fire and forget)", target_controller.c_str());
        return true;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Controller switch service not ready");
        return false;
    }
}

// Velocity timer callback - hybrid approach: integrate velocity to position
void FakeAxisCamera::velocity_timer_callback()
{
    // Only operate in velocity mode
    if (mode_ != ptz_action_server_msgs::msg::PtzState::MODE_VELOCITY) {
        return;
    }
    
    // Make sure position_controller is active (we use it for real movement even in velocity mode)
    if (current_active_controller_ != "position_controller") {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Velocity mode but controller is: %s (expected position_controller)", 
                             current_active_controller_.c_str());
        return;
    }
    
    // HYBRID APPROACH: Use position_controller for real movement, integrate velocities manually
    
    // 1. Integrate velocities into current positions  
    current_pan_ += current_vel_pan_ * dt;
    current_tilt_ += current_vel_tilt_ * dt;
    
    // Clamp positions to limits
    current_pan_ = clamp(current_pan_, PAN_MIN_, PAN_MAX_);
    current_tilt_ = clamp(current_tilt_, TILT_MIN_, TILT_MAX_);
    
    // 2. Send integrated positions to position_controller for actual movement
    std_msgs::msg::Float64MultiArray position_msg;
    position_msg.data.push_back(current_pan_);
    position_msg.data.push_back(current_tilt_);
    pos_cmd_pub_->publish(position_msg);
    
    static int debug_count = 0;
    if (++debug_count % 50 == 0) {  // Log every 1 seconds
        RCLCPP_INFO(this->get_logger(), "Velocity mode: integrating vel=[%.3f,%.3f] -> pos=[%.3f,%.3f]", 
                   current_vel_pan_, current_vel_tilt_, current_pan_, current_tilt_);
    }

    // Real zoom control via CameraZoomPlugin - integrate velocity to zoom factor
    if (std::abs(current_vel_zoom_) > 0.001f) {
        current_zoom_ += current_vel_zoom_ * dt;
        current_zoom_ = clamp(current_zoom_, ZOOM_MIN_, ZOOM_MAX_);
        
        // Send zoom command to real CameraZoomPlugin
        std_msgs::msg::Float64 zoom_msg;
        zoom_msg.data = current_zoom_;
        ptz_zoom_cmd_pub_->publish(zoom_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "🔍 REAL ZOOM: Velocity=%.3f -> zoom=%.3f", 
                    current_vel_zoom_, current_zoom_);
    }
    
    // NEW: Publish joint states for both RViz and Gazebo
    publish_joint_states();
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
        
        // NEW: Publish position commands for both RViz and Gazebo
        std_msgs::msg::Float64MultiArray position_msg;
        position_msg.data.push_back(current_pan_);
        position_msg.data.push_back(current_tilt_);
        pos_cmd_pub_->publish(position_msg);
        
        // NEW: Publish joint states for RViz and PTZ state
        publish_joint_states();
        
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

// Function to force joint positions directly in Gazebo (bypass mock_components)
void FakeAxisCamera::force_gazebo_joint_positions(float pan_rad, float tilt_rad)
{
    // Publish joint positions directly to Gazebo joint command topics
    // This bypasses the ROS2 Control layer entirely
    
    auto pan_msg = std_msgs::msg::Float64();
    auto tilt_msg = std_msgs::msg::Float64();
    
    pan_msg.data = pan_rad;
    tilt_msg.data = tilt_rad;
    
    pan_joint_pub_->publish(pan_msg);
    tilt_joint_pub_->publish(tilt_msg);
    
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter % 50 == 0) { // Debug every 1 second at 50Hz
        RCLCPP_INFO(this->get_logger(), "🔥 COMMANDING Gazebo via /pan_cmd & /tilt_cmd: pan=%.3f, tilt=%.3f rad", 
            pan_rad, tilt_rad);
    }
}
