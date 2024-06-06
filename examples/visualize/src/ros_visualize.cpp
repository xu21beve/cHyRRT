#include <rclcpp/rclcpp.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <visualization_msgs/msg/marker.hpp>
// #include "../include/rrtImpl.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>

std::vector<std::vector<double>> make_matrix()
{
    std::ifstream file("src/points.txt");
    std::string line;
    std::vector<std::vector<double>> matrix;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::vector<double> row;
        double value;
        while (iss >> value)
        {
            row.push_back(value);
        }
        matrix.push_back(row);
    }

    std::cout << "Number of points: " << matrix.size() << std::endl;
    return matrix;
}

class PointMatrixPublisher : public rclcpp::Node
{
public:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    PointMatrixPublisher(std::vector<std::vector<double>> matrix) : Node("point_matrix_publisher")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("point_matrix", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("obstacles", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this, matrix]()
                                         { this->publishPoints(matrix); });
    }

private:
    void publishPoints(std::vector<std::vector<double>> matrix)
    {
        auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();

        size_t rows = matrix.size();

        visualization_msgs::msg::Marker marker, lines;
        marker.header.frame_id = "map"; // Change to your desired frame
        lines.header.frame_id = "map";

        marker.ns = "points";
        lines.ns = "points";

        for (size_t row = 0; row < rows; ++row)
        {
            marker.header.stamp = this->now();
            lines.header.stamp = this->now();

            marker.id = row;
            lines.id = row + 3102;

            marker.pose.position.x = matrix[row][3]; // Assuming x-coordinate is in the first column
            marker.pose.position.y = matrix[row][0]; // Assuming y-coordinate is the row index
            marker.pose.position.z = 0.0;

            if (row == 0)
            {
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.scale.x = 0.15;
                marker.scale.y = 0.15;
                marker.scale.z = 0.05;
            }
            else
            {
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;
                if (row > 10) {
                    marker.color.a = 0.5;
                    marker.color.r = 1.0;
                    marker.color.g = 1.0;
                    marker.color.b = 1.0;
                }
                else if(row ==10) {
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    marker.color.a = 1.0;
                    marker.color.r = 0.7;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                    marker.scale.x = 0.15;
                    marker.scale.y = 0.15;
                    marker.scale.z = 0.05;
                }
                else {
                    marker.type = visualization_msgs::msg::Marker::CUBE;
                    marker.scale.x = 0.10;
                    marker.scale.y = 0.10;
                    marker.scale.z = 0.10;
                    if(row == 4 || row == 5 || row == 7 || row== 8) {
                        marker.color.a = 1.0;
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                    }
                    else {
                        marker.color.a = 1.0;
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 1.0;
                    }
                }
                // else if(matrix[row][1] < 1.5 && matrix[row][1] > 1.0 && matrix[row][0] < 4.5){
                //     marker.color.a = 0.0;
                //     marker.color.r = 1.0;
                //     marker.color.g = 1.0;
                //     marker.color.b = 1.0;
                // }
                // else {
                    
                // }
                
            }

            // if (matrix[row][0] < 0.0 && row != rows - 1)
            // {
            //     std::cout << "line added" << std::endl;
            //     lines.type = visualization_msgs::msg::Marker::LINE_STRIP;
            //     lines.action = visualization_msgs::msg::Marker::ADD;
            //     lines.color.a = 1.0;
            //     lines.color.r = 1.0;
            //     lines.color.g = 0.0;
            //     lines.color.b = 0.0;
            //     lines.scale.x = 0.05;

            //     geometry_msgs::msg::Point p1, p2;
            //     p1.x = matrix[row][0];
            //     p1.y = matrix[row][1];

            //     p2.x = matrix[row + 1][0];
            //     p2.y = matrix[row + 1][1];

            //     lines.points.push_back(p1);
            //     lines.points.push_back(p2);
            // }

            marker_array_msg->markers.push_back(marker);
            marker_array_msg->markers.push_back(lines);
        }
        visualization_msgs::msg::Marker obs1, obs2, obs3, obs4;
        rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("talker");
        rclcpp::Time now = node->get_clock()->now();
        rclcpp::Duration lifetime_time(0, 0); // 0 makes it forever

        obs1.type = obs2.type = obs3.type = obs4.type = visualization_msgs::msg::Marker::CUBE;
        obs1.header.frame_id = obs2.header.frame_id = obs3.header.frame_id = obs4.header.frame_id = "/map";

        obs1.header.stamp = obs2.header.stamp = obs3.header.stamp = obs4.header.stamp = now;
        obs1.ns = obs2.ns = obs3.ns = obs4.ns = "obstacles";
        obs1.lifetime = obs2.lifetime = obs3.lifetime = obs4.lifetime = lifetime_time;
        obs1.action = obs2.action = obs3.action = obs4.action = visualization_msgs::msg::Marker::ADD;

        obs1.id = 0; // initialized at beginning
        obs2.id = 1; // initialized after 1500 msgs
        obs3.id = 2; // initialized after 900 msgs
        obs4.id = 3; // initialized after _____ msgs

        obs1.scale.x = obs2.scale.x = 4.5;
        obs1.scale.y = obs2.scale.y = 0.5;
        obs3.scale.y = 1;
        obs3.scale.x = 0.5;
        obs4.scale.x = 2;
        obs4.scale.y = 0.5;

        obs1.scale.z = obs2.scale.z = obs3.scale.z = obs4.scale.z = 0;

        obs1.pose.position.x = obs1.scale.x / 2; // flat 2x12 rect @ (8, 6)
        obs1.pose.position.y = 3 - obs1.scale.x / 2 + 2;
        // obs1.pose.position.z = 0.25;
        obs1.pose.orientation.x = 0.0;
        obs1.pose.orientation.y = 0.0;
        obs1.pose.orientation.z = 0.0;
        obs1.pose.orientation.w = 1;
        obs1.color.a = 1;
        obs1.color.r = obs1.color.g = obs1.color.b = 255.0f;

        obs2.pose.position.x = obs2.scale.x / 2; // flat 2x12 rect @ (14, 14)
        obs2.pose.position.y = 1.5 - obs2.scale.x / 2 + 2;
        // obs2.pose.position.z = 0.25;
        obs2.pose.orientation.x = 0.0;
        obs2.pose.orientation.y = 0.0;
        obs2.pose.orientation.z = 0.0;
        obs2.pose.orientation.w = 1;
        obs2.color.a = 1;
        obs2.color.r = obs2.color.g = obs2.color.b = 255.0f;

        obs3.pose.position.x = obs3.scale.x / 2; // flat 7x4 rect @ (16.5, 2)
        obs3.pose.position.y = .5 - obs3.scale.y / 2 + 2;
        // obs3.pose.position.z = 0.25;
        obs3.pose.orientation.x = 0.0;
        obs3.pose.orientation.y = 0.0;
        obs3.pose.orientation.z = 0.0;
        obs3.pose.orientation.w = 1;
        obs3.color.a = 1;
        obs3.color.r = obs3.color.g = obs3.color.b = 255.0f;

        obs4.pose.position.x = 1;
        obs4.pose.position.y = 1;
        obs4.pose.orientation.x = 0.0;
        obs4.pose.orientation.y = 0.0;
        obs4.pose.orientation.z = 0.0;
        obs4.color.a = 1;
        obs4.color.r = obs4.color.g = obs4.color.b = 255.0f;

        marker_pub->publish(obs1);
        marker_pub->publish(obs2);
        marker_pub->publish(obs3);

        // obsVec.push_back(obs1);
        // obsVec.push_back(obs2);
        // obsVec.push_back(obs3);

        marker_pub_->publish(*marker_array_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Assuming you have a matrix named "matrix" with appropriate dimensions
    std::vector<std::vector<double>> matrix = make_matrix();
    // populateRviz(publisher);
    rclcpp::spin(std::make_shared<PointMatrixPublisher>(matrix));
    // rclcpp::shutdown();

    return 0;
}