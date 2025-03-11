//
// Created by wuqilin on 25-3-11.
//
#include "phigros_clicker.hpp"

PhigrosClicker::PhigrosClicker() : Node("phigros_clicker") {  //创建节点“phigros_clicker"
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/raw_image" , 10, std::bind(&PhigrosClicker::image_callback, this, std::placeholders::_1));
    //this指针指的是phigros_clicker"这个节点，"/raw_image"是话题名称，10是队列大小，
    // 使用std::bind函数把this和回调函数绑定到一起，std::placeholder::_1是占位符，表示传入的参数，即收到的消息
    pos_pub_ = this->create_publisher<geometry_msgs::msg::Point32>(
        "/click_position", 10);
}

void PhigrosClicker::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "image received");
    try
    {
      cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image; // 转换为 OpenCV Matcv_bridge 是 ROS2 和 OpenCV 之间的桥梁，用于将 sensor_msgs::msg::Image 与 cv::Mat 相互转换。
      //toCvCopy(msg, "bgr8") 作用：

          //msg：ROS2 传入的 sensor_msgs::msg::Image（即订阅的原始图像）。
          //"bgr8"：指定图像的编码格式，表示 8位深度 BGR（蓝-绿-红）格式，OpenCV 默认使用 BGR 格式。
          //返回一个 cv_bridge::CvImage::Ptr（智能指针），指向转换后的 OpenCV 图像对象。

      int total_pixels = img.total();//获取总像素数
      geometry_msgs::msg::Point32 click_pos;
      RCLCPP_INFO(this->get_logger(), "成功转换图像, 尺寸: %dx%d", img.rows, img.cols);
      RCLCPP_INFO(this->get_logger(), "像素个数: %d", total_pixels);
      cv::Mat hsv;
      cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);//转换为HSV颜色空间
      //高斯模糊处理
      cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 0);
      //img：输入图像
      //hsv:输出图像，转换后的HSV
      //cv::COLOR_BGR2HSV表示颜色空间从BGR转换为HSV
      cv::Mat mask;
      cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255), mask);//提取白色区域
      //hsv:输入的图像，cv::Scalar(0, 0, 200), cv::Scalar(180, 30, 255)分别是颜色的上下限，mask是输出的图像，是二值掩码图像
      cv::Mat edges;
      cv::Canny(mask, edges, 50, 150);//边缘检测，mask是输入图像，edges是输出的边缘图像，也是二值图像
      //50（低阈值）：小于此值的像素会被直接 丢弃，不会被认为是边缘。
		//150（高阈值）：大于此值的像素会被直接 保留，认为是边缘。

	  cv::Mat blue_mask;
      cv::inRange(hsv, cv::Scalar(90, 100, 100), cv::Scalar(120, 255, 255), blue_mask);//提取蓝色区域
      std::vector <std::vector<cv::Point> > contours;//输出的轮廓列表，每个轮廓都是std::vector<cv::Point>
      cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);//cv::RETR_EXTERNAL只提取最外层的轮廓，忽略嵌套的轮廓

	  float cx;
      float cy;
      //遍历所有轮廓
      for (const auto &contour : contours) {
        cv::Moments m = cv::moments(contour, true);
        if(m.m00 > 0){
        	cx = int(m.m10 / m.m00);//计算x方向的质心
        	cy = int(m.m01 / m.m00);//计算y方向的质心

        	std::cout << "蓝色区域中心：（" << cx << "," << cy << ")" << std::endl;

            blue_center_.push_back(cv::Point2f(cx, cy));//存入中心列表
            std::sort(blue_center_.begin(), blue_center_.end(), [](const cv::Point2i &a, const cv::Point2i &b) {
              return a.y > b.y;
            });
      	}
      	//m.m00	区域面积（像素数）
        //m.m10	计算重心 X 方向的矩
        //m.m01	计算重心 Y 方向的矩
        //m.m11	计算区域的倾斜程度
        //m.m20	计算水平方向的分布
        //m.m02	计算垂直方向的分布
      }

      cv::HoughLinesP(edges, lines, 1.0, CV_PI/180.0, 100, 500, 30);
      	//edges	经过 Canny 边缘检测 处理的 二值图
		//lines	输出的 直线集合，每条直线由 四个整数 (x1, y1, x2, y2) 组成
		//1	累加器分辨率 (rho)，单位是 像素，即直线距离的最小步长
		//CV_PI / 180	角度分辨率 (theta)，单位是 弧度，这里设置为 1°
		//100	投票阈值（最小投票数，越大则检测出的直线越少，但质量越高）
		//500	最短直线长度（低于这个长度的直线会被忽略）
		//30	最大间断长度（间断部分小于这个值的直线会被合并）

      for (size_t i = 0; i < lines.size(); i++)
      {
        cv::Vec4i l= lines[i];
        std::cout << "Line:" << l[0] << " " << l[1] << " " << l[2] << " " << l[3] << std::endl;
        lines_center_[0].x = (l[0] + l[2]) / 2;//计算判定线的中心坐标
        lines_center_[0].y = (l[1] + l[3]) / 2;
        std::cout << "Line_point" << lines_center_[0].x << " " << lines_center_[0].y << std::endl;

      }

      if(lines_center_[0].y - blue_center_[0].y <= 22.0 )
      {

        click_pos.x = blue_center_[0].x;
        click_pos.y = blue_center_[0].y;
        click_pos.z = 0;
        //发布点击位置
        pos_pub_->publish(click_pos);
        RCLCPP_INFO(this->get_logger(), "发布点击位置:(%2f, %2f)", click_pos.x, click_pos.y);
		blue_center_.erase(blue_center_.begin());
      }
	  for (const auto &l : lines) {
            int x1= l[0];
            int y1 = l[1];
            int x2 = l[2];
            int y2 = l[3];
            //计算角度（以弧度转换为单位）
            double angle_rad = std::atan2((y2 - y1), (x2 - x1));
            //转换角度
            double angle_deg ;
            if(x1 == x2)
            {
              angle_deg = 90.0;
            }
            else{
              angle_deg = angle_rad *180.0/M_PI;
            }
            //存储角度
            line_angles_.push_back(angle_deg);
            std::cout << angle_deg << std::endl;
            //角度只存一个就够了
            if(line_angles_.size() > 1.0)
            {
              line_angles_.erase(line_angles_.begin());//及时更新角度信息
            }
	  }

    }

    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "图像转换失败: %s", e.what());
    }


  }