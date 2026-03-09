#!/bin/bash

# PTZ Navigation Stack Integration Test Script
# Tests the integration between hardware motion stack and simulation

echo "=== PTZ Navigation Stack Integration Test ==="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Helper function for colored output
print_status() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $2"
    else
        echo -e "${RED}✗${NC} $2"
        echo -e "${YELLOW}  Check: $3${NC}"
    fi
}

# Check if simulation is running
echo "1. Checking simulation status..."
ros2 topic list | grep -q "/joint_states"
print_status $? "Simulation running" "Launch: ros2 launch ptz_gz_sim spawn_ptz_in_rover_world.launch.py"

# Check if ptz_manager is running
echo "2. Checking ptz_manager status..."
ros2 node list | grep -q "ptz_manager"
print_status $? "PTZ manager running" "Launch: ros2 launch ptz_gz_sim ptz_navigation_stack.launch.py"

# Check PTZ action server
echo "3. Checking PTZ action server..."
ros2 action list | grep -q "move_ptz/position_abs"
print_status $? "Action server available" "fake_axis_camera_node should be running"

# Check controller manager
echo "4. Checking controllers..."
controllers=$(ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers --timeout=2 2>/dev/null)
if echo "$controllers" | grep -q "position_controller"; then
    print_status 0 "Controllers active" ""
else  
    print_status 1 "Controllers not found" "Check controller_manager and ros2_control"
fi

# Test velocity control
echo ""
echo "5. Testing velocity control..."
echo "Publishing velocity command..."
ros2 topic pub --once /cmd/velocity ptz_action_server_msgs/msg/Ptz "{pan: 0.1, tilt: 0.05, zoom: 0.0}" &
sleep 2

# Check if position is changing
pos1=$(ros2 topic echo /joint_states --once 2>/dev/null | grep -A1 "position:" | tail -n1)
sleep 1
pos2=$(ros2 topic echo /joint_states --once 2>/dev/null | grep -A1 "position:" | tail -n1)

if [ "$pos1" != "$pos2" ]; then
    print_status 0 "Velocity control working" ""
else
    print_status 1 "Velocity control not working" "Check fake_axis_camera logs"
fi

# Stop velocity
echo "Stopping velocity..."
ros2 topic pub --once /cmd/velocity ptz_action_server_msgs/msg/Ptz "{pan: 0.0, tilt: 0.0, zoom: 0.0}"

echo ""
echo "6. Testing position control..."
echo "Publishing position command..."
ros2 topic pub --once cmd/ptz ptz_action_server_msgs/msg/Ptz "{pan: 0.5, tilt: 0.3, zoom: 5.0}" &
sleep 3

# Check if reached target approximately
current_pos=$(ros2 topic echo /joint_states --once 2>/dev/null | grep -A3 "position:" | tail -n1)
if echo "$current_pos" | grep -q "0\.[3-7]"; then
    print_status 0 "Position control working" ""
else
    print_status 1 "Position control not working" "Check action server and joint_states topic"
fi

echo ""
echo "7. Testing SEED commands..."
echo "Publishing cover command..."
ros2 topic pub --once /seed_pdt_camera/command std_msgs/msg/String "data: 'cover(null,34,1,0:01:00)'" &

# Check if ptz_manager processes the command
sleep 2
ptz_manager_logs=$(ros2 topic echo /rosout --timeout=1 2>/dev/null | grep -i "cover\|task")
if [ -n "$ptz_manager_logs" ]; then
    print_status 0 "SEED cover command processed" ""
else
    print_status 1 "SEED command not processed" "Check ptz_manager logs and parameters"
fi

echo ""
echo "8. Topic remapping verification..."

# Check key topics exist
topics_to_check=("/ptz_state" "/cmd/velocity" "/seed_pdt_camera/command")
all_topics_ok=true

for topic in "${topics_to_check[@]}"; do
    ros2 topic list | grep -q "^${topic}$"
    if [ $? -eq 0 ]; then
        echo -e "  ${GREEN}✓${NC} $topic exists"
    else
        echo -e "  ${RED}✗${NC} $topic missing"
        all_topics_ok=false
    fi
done

if $all_topics_ok; then
    print_status 0 "Topic remapping correct" ""
else
    print_status 1 "Topic remapping issues" "Check launch file remappings"
fi

echo ""
echo "=== Integration Test Summary ==="
echo ""
echo "Manual test commands:"
echo "# Velocity control:"
echo "ros2 topic pub --once /cmd/velocity ptz_action_server_msgs/msg/Ptz '{pan: 0.1, tilt: 0.05, zoom: 0.0}'"
echo ""
echo "# Position control:"
echo "ros2 topic pub --once cmd/ptz ptz_action_server_msgs/msg/Ptz '{pan: 0.5, tilt: 0.3, zoom: 5.0}'"
echo ""
echo "# SEED coverage:"
echo "ros2 topic pub --once /seed_pdt_camera/command std_msgs/msg/String \"data: 'cover(null,34,1,0:05:00)'\""
echo ""
echo "# Monitor topics:"
echo "ros2 topic echo /ptz_state"
echo "ros2 topic echo /joint_states" 
echo ""
echo -e "${YELLOW}Note: Make sure both simulation and navigation stack are running!${NC}"