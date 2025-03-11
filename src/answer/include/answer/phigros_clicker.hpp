//
// Created by wuqilin on 25-3-11.
//

#ifndef PHIGROS_CLICKER_HPP
#define PHIGROS_CLICKER_HPP

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point32.hpp"
#include <cmath>
#include <cv_bridge/cv_bridge.h>
class PhigrosClicker : public rclcpp::Node {  //创建类PhigrosClicker
public:
    PhigrosClicker();
private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;//创建一个订阅者image_sub_，用来订阅“raw_image”话题
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr pos_pub_;//创建一个发布者pos_pub_, 用来发布消息到“/click position"上
    std::vector<cv::Vec4i> lines;//存储直线信息
    std::vector<cv::Point2f> lines_center_ = {cv::Point2f(0,0)};//存储直线的中心位置
    std::vector<double> line_angles_;//存储直线与水平线的夹角
    std::vector<cv::Point2f> blue_center_ ;//存储蓝条的中心位置
};
#endif //PHIGROS_CLICKER_HPP
