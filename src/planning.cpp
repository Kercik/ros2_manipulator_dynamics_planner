#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cctype>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("advanced_planning_node");
    
    // Create executor for proper node handling
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor]() { executor.spin(); });
    
    try {
        // Create MoveGroupInterface for ur_manipulator group
        moveit::planning_interface::MoveGroupInterface move_group(
            node, "ur_manipulator",
            std::make_shared<tf2_ros::Buffer>(node->get_clock()),
            std::chrono::seconds(10)
        );
        
        // Main loop
        while (rclcpp::ok()) {
            // Get planner selection
            std::string planner_id;
            std::cout << "\nWybierz planer (RRTConnect, PRM, RRT, TRRT, default: RRTConnect): ";
            std::getline(std::cin, planner_id);
            if (planner_id.empty()) planner_id = "RRTConnect";
            
            // Get motion type
            std::string motion_type;
            std::cout << "Wybierz typ ruchu (lin/ptp, default: ptp): ";
            std::getline(std::cin, motion_type);
            if (motion_type.empty()) motion_type = "ptp";
            
            // Convert to lowercase for easier comparison
            for (char& c : motion_type) c = std::tolower(c);
            
            // Get target position
            double x, y, z, roll = 0.0, pitch = 0.0, yaw = 0.0;
            std::cout << "Podaj pozycję docelową (x y z): ";
            std::cin >> x >> y >> z;
            
            // Get orientation if needed
            if (motion_type == "lin") {
                std::cout << "Podaj orientację (roll pitch yaw w radianach): ";
                std::cin >> roll >> pitch >> yaw;
            }
            std::cin.ignore();  // Clear input buffer
            
            // Set planner
            move_group.setPlannerId(planner_id);
            RCLCPP_INFO(node->get_logger(), "Używam planera: %s", planner_id.c_str());
            
            // Set target pose in world frame
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = "world";
            target_pose.pose.position.x = x;
            target_pose.pose.position.y = y;
            target_pose.pose.position.z = z;
            
            // Convert RPY to quaternion
            tf2::Quaternion orientation;
            orientation.setRPY(roll, pitch, yaw);
            target_pose.pose.orientation = tf2::toMsg(orientation);
            
            RCLCPP_INFO(node->get_logger(), "Pozycja docelowa:");
            RCLCPP_INFO(node->get_logger(), "  X: %.2f, Y: %.2f, Z: %.2f", x, y, z);
            RCLCPP_INFO(node->get_logger(), "  Orientacja: R=%.2f, P=%.2f, Y=%.2f", roll, pitch, yaw);
            
            if (motion_type == "lin") {
                RCLCPP_INFO(node->get_logger(), "Planowanie ruchu liniowego...");
                
                // Plan Cartesian path
                std::vector<geometry_msgs::msg::Pose> waypoints;
                waypoints.push_back(target_pose.pose);
                
                moveit_msgs::msg::RobotTrajectory trajectory;
                const double step = 0.01;  // Step size (meters)
                const double jump_threshold = 0.0;  // Disable jump check
                double fraction = move_group.computeCartesianPath(
                    waypoints, step, jump_threshold, trajectory);
                
                if (fraction >= 0.9) {
                    RCLCPP_INFO(node->get_logger(), "Planowanie liniowe udane (%.1f%%). Wykonuję...", fraction * 100.0);
                    
                    // Create a plan object from the trajectory
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory;
                    
                    auto execute_result = move_group.execute(plan);
                    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_INFO(node->get_logger(), "Wykonanie zakończone sukcesem!");
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Błąd wykonania: %d", execute_result.val);
                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Planowanie liniowe nieudane (tylko %.1f%% ścieżki).", fraction * 100.0);
                }
            } else {
                RCLCPP_INFO(node->get_logger(), "Planowanie w przestrzeni jointów...");
                
                // Set goal for tool0 end effector
                move_group.setPoseTarget(target_pose, "tool0");
                
                // Plan to target
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                auto plan_result = move_group.plan(plan);
                
                if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    RCLCPP_INFO(node->get_logger(), "Planowanie udane! Wykonuję...");
                    auto execute_result = move_group.execute(plan);
                    
                    if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                        RCLCPP_INFO(node->get_logger(), "Wykonanie zakończone sukcesem!");
                    } else {
                        RCLCPP_ERROR(node->get_logger(), "Błąd wykonania: %d", execute_result.val);
                    }
                } else {
                    RCLCPP_ERROR(node->get_logger(), "Błąd planowania: %d", plan_result.val);
                }
            }
            
            // Ask to continue
            std::cout << "\nCzy chcesz zaplanować kolejny ruch? (t/n): ";
            std::string response;
            std::getline(std::cin, response);
            
            if (response != "t" && response != "T") {
                break;
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Wyjątek: %s", e.what());
    }
    
    // Shutdown
    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}