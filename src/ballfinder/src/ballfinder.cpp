#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <ros/topic.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>

// #define TROUBLESHOOT

const auto KINECT_ENCODING = sensor_msgs::image_encodings::TYPE_16UC1;

struct CameraInstrinsics
{
    double fx;
    double fy;
    double cx;
    double cy;
};

struct ColorRange
{
    cv::Scalar lower;
    cv::Scalar upper;

    void print()
    {
        std::cout << "Lower: " << lower << std::endl;
        std::cout << "Upper: " << upper << std::endl;
    }
};

class BallFinder
{
public:
    BallFinder(ros::NodeHandle &nh) : nh_(nh), previous_point_(0, 0, 0), ns_(ros::this_node::getNamespace())
    {

        loadParams();
        printParams();
        initROS();
        fetchCameraToWorldTransform();
        camera_intrinsics_ = fetchCameraIntrinsics();

    }

private:
    
    void initROS() {
        
        if (ros::param::has("rgb_image_topic")) {
            ros::param::get("rgb_image_topic", rgb_image_topic_);
        }
        else {
            throw std::runtime_error("No rgb_image_topic specified in the parameter server!");
        }

        if (ros::param::has("depth_image_topic")) {
            ros::param::get("depth_image_topic", depth_image_topic_);
        }
        else {
            throw std::runtime_error("No depth_image_topic specified in the parameter server!");
        }

        if (ros::param::has("camera_info_topic")) {
            ros::param::get("camera_info_topic", camera_info_topic_);
        }
        else {
            throw std::runtime_error("No camera_info_topic specified in the parameter server!");
        }

        if (ros::param::has("camera_frame")) {
            ros::param::get("camera_frame", camera_frame_id_);
        }
        else {
            throw std::runtime_error("No camera_frame specified in the parameter server!");
        }

        rgb_image_subscriber_ = nh_.subscribe(rgb_image_topic_, 10, &BallFinder::rgbImageCallback, this);
        depth_image_subscriber_ = nh_.subscribe(depth_image_topic_, 10, &BallFinder::depthImageCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(ns_ + "/state", 10);
        position_publisher_ = nh_.advertise<geometry_msgs::PointStamped>(ns_ + "/position", 10);
    }

    void fetchCameraToWorldTransform() {
        tf::Transform transform;
        ROS_WARN_STREAM("Waiting for transform from " << camera_frame_id_ << " to /world");
        listener_.waitForTransform("/world", camera_frame_id_, ros::Time(0), ros::Duration(10.0));
    }

    void loadParams()
    {
        loadColorRanges();

        gaussian_blur_kernel_size_ = 5;
        if (ros::param::has("gaussian_blur_kernel_size"))
        {
            ros::param::get("gaussian_blur_kernel_size", gaussian_blur_kernel_size_);
        }

        do_gaussian_blur_ = true;
        if (ros::param::has("do_gaussian_blur"))
        {
            ros::param::get("do_gaussian_blur", do_gaussian_blur_);
        }

        do_histogram_equalization_ = true;
        if (ros::param::has("do_histogram_equalization"))
        {
            ros::param::get("do_histogram_equalization", do_histogram_equalization_);
        }

    }

    void printParams()
    {
        std::cout << "=========== Parameters: =============" << std::endl;
        std::cout << "Namespace: " << ns_ << std::endl;
        std::cout << "Do gaussian blur: " << do_gaussian_blur_ << std::endl;
        std::cout << "Gaussian blur kernel size: " << gaussian_blur_kernel_size_ << std::endl;
        std::cout << "Do histogram equalization: " << do_histogram_equalization_ << std::endl;
        for (auto &cr : color_ranges_)
        {
            cr.print();
        }
    }

    void loadColorRanges()
    {

        // Loading HSV color ranges
        if (!ros::param::has("hsv_color_ranges"))
        {
            ROS_ERROR_STREAM("No HSV color ranges specified in the parameter server!");
            return;
        }

        // Getting the color ranges
        XmlRpc::XmlRpcValue color_ranges;
        ros::param::get("hsv_color_ranges", color_ranges);
        ROS_ASSERT(color_ranges.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int i = 0; i < color_ranges.size(); ++i)
        {
            ROS_ASSERT(color_ranges[i].getType() == XmlRpc::XmlRpcValue::TypeStruct);
            ROS_ASSERT(color_ranges[i].size() == 3);
            ROS_ASSERT(color_ranges[i].hasMember("h"));
            ROS_ASSERT(color_ranges[i].hasMember("s"));
            ROS_ASSERT(color_ranges[i].hasMember("v"));
            ROS_ASSERT(color_ranges[i]["h"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(color_ranges[i]["s"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(color_ranges[i]["v"].getType() == XmlRpc::XmlRpcValue::TypeArray);
            ROS_ASSERT(color_ranges[i]["h"].size() == 2);
            ROS_ASSERT(color_ranges[i]["s"].size() == 2);
            ROS_ASSERT(color_ranges[i]["v"].size() == 2);

            ColorRange cr;
            cr.lower = cv::Scalar(
                static_cast<int>(color_ranges[i]["h"][0]),
                static_cast<int>(color_ranges[i]["s"][0]),
                static_cast<int>(color_ranges[i]["v"][0]));
            cr.upper = cv::Scalar(
                static_cast<int>(color_ranges[i]["h"][1]),
                static_cast<int>(color_ranges[i]["s"][1]),
                static_cast<int>(color_ranges[i]["v"][1]));
            color_ranges_.push_back(std::move(cr));
        }
    }

    CameraInstrinsics fetchCameraIntrinsics()
    {
        ROS_INFO_STREAM("Waiting for camera info...");
        auto camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, nh_);
        auto K = camera_info->K.data();
        ROS_INFO_STREAM("Got camera parameters.\n");
        return CameraInstrinsics{K[0], K[4], K[2], K[5]};
    }

    void rgbImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {

        try
        {
            cv_ptr_bgr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception at rgb: %s", e.what());
            return;
        }

#ifdef TROUBLESHOOT
        cv::imshow("RGB_WINDOW", cv_ptr_bgr_->image);
        cv::waitKey(3);
#endif

        // Getting transform from camera frame to world frame
        tf::StampedTransform transform;
        try
        {
            listener_.lookupTransform("/world", camera_frame_id_, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }

        // Checking if we have a depth image
        if (!cv_ptr_depth_)
        {
            // ROS_ERROR("No depth image received yet! Skipping.\n");
            return;
        }

        // Applying image preprocessing
        preprocessImage(cv_ptr_bgr_->image);

        // Applying image segmentation
        cv::Mat mask = segmentImage(cv_ptr_bgr_->image);

        // Finding contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            ROS_WARN_STREAM("No red objects found!\n");
            return;
        }

        auto biggest_bb = findLargestBoundingBox(contours);
        auto bb_center = computeBoundingBoxCenter(biggest_bb);
        auto position = reconstructCenterPoint(bb_center); 
        
        // Converting point to world frame
        tf::Point tf_position(position.x, position.y, position.z);
        tf_position = transform * tf_position;

        position = cv::Point3d(tf_position.x(), tf_position.y(), tf_position.z());

        // Compute velocity
        auto velocity = computeVelocity(cv::Point3d(tf_position.x(), tf_position.y(), tf_position.z()));

        
        // Publish position as point
        geometry_msgs::PointStamped point;
        point.header.stamp = ros::Time::now();
        point.header.frame_id = "world";
        point.point.x = position.x;
        point.point.y = position.y;
        point.point.z = position.z;
        position_publisher_.publish(point);

        // Publish the position and velocity with Odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";

        odom.pose.pose.position.x = position.x;
        odom.pose.pose.position.y = position.y;
        odom.pose.pose.position.z = position.z;

        odom.twist.twist.linear.x = velocity.x;
        odom.twist.twist.linear.y = velocity.y;
        odom.twist.twist.linear.z = velocity.z;

        odom_pub_.publish(odom);



#ifdef TROUBLESHOOT
        cv::rectangle(cv_ptr_bgr_->image, biggest_bb, cv::Scalar(0, 255, 0), 2);
        cv::circle(cv_ptr_bgr_->image, bb_center, 5, cv::Scalar(0, 0, 255), -1);
        cv::imshow("BOUNDING BOX", cv_ptr_bgr_->image);
        cv::waitKey(3);
        ROS_INFO_STREAM("Position: " << position << "\n");
        ROS_INFO_STREAM("Velocity: " << velocity << "\n");
#endif
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        try
        {
            cv_ptr_depth_ = cv_bridge::toCvCopy(msg, KINECT_ENCODING);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception at depth: %s", e.what());
            return;
        }

#ifdef TROUBLESHOOT
        cv::imshow("DEPTH_WINDOW", cv_ptr_depth_->image);
        cv::waitKey(3);
#endif
    }

    cv::Rect findLargestBoundingBox(const std::vector<std::vector<cv::Point>> &contours)
    {
        size_t biggest_countour_ix = 0;
        double biggest_countour_area = 0;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = cv::contourArea(contours[i]);
            if (area > biggest_countour_area)
            {
                biggest_countour_area = area;
                biggest_countour_ix = i;
            }
        }
        return cv::boundingRect(contours[biggest_countour_ix]);
    }

    cv::Point2d computeBoundingBoxCenter(const cv::Rect &bb)
    {
        return cv::Point2d(bb.x + bb.width / 2, bb.y + bb.height / 2);
    }

    cv::Point3d reconstructCenterPoint(const cv::Point2d &center)
    {

        size_t u = center.x;
        size_t v = center.y;

        // ROS_WARN_STREAM("Dereferencing depth image...");
        double z = cv_ptr_depth_->image.at<uint16_t>(v, u) / 1000.0; // mm -> m
        ROS_WARN_STREAM("Depth: " << z << "\n");

        double x = z * (u - camera_intrinsics_.cx) / camera_intrinsics_.fx;
        double y = z * (v - camera_intrinsics_.cy) / camera_intrinsics_.fy;
        
        // Print x y z
        ROS_WARN_STREAM("x: " << x << " " << "" << "y: " << y << " " << "" << "z: " << z << "\n");


        return cv::Point3d(x, y, z);
    }

    cv::Point3d computeVelocity(const cv::Point3d &point_3d)
    {
        double dt = (ros::Time::now() - last_time_).toSec();
        last_time_ = ros::Time::now();

        cv::Point3d velocity = (point_3d - previous_point_) / dt;
        previous_point_ = point_3d;

        return velocity;
    }

    void preprocessImage(const cv::Mat &img)
    {

        if (do_gaussian_blur_)
        {
            cv::GaussianBlur(img, img, cv::Size(gaussian_blur_kernel_size_, gaussian_blur_kernel_size_), 0, 0);
        }

        //
        // Histogram equalization
        //
        if (do_histogram_equalization_)
        {
            cv::cvtColor(img, img, cv::COLOR_BGR2YUV);
            cv::Mat Y_channel;
            cv::extractChannel(img, Y_channel, 0);
            cv::equalizeHist(Y_channel, Y_channel);
            cv::cvtColor(img, img, cv::COLOR_YUV2BGR);
        }
         


#ifdef TROUBLESHOOT
        cv::imshow("PROCESSED_RGB", cv_ptr_bgr_->image);
        cv::waitKey(3);
#endif
    }

    cv::Mat segmentImage(const cv::Mat &img)
    {

        // Performing color-based color segmentation
        cv::cvtColor(img, img, cv::COLOR_BGR2HSV);

        cv::Mat mask(img.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat curr_mask;
        for (const auto &color_range : color_ranges_)
        {
            cv::inRange(img, color_range.lower, color_range.upper, curr_mask);
            mask |= curr_mask;
        }

        // Apply erosion and dilation
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 3);



#ifdef TROUBLESHOOT
        cv::imshow("MASK", mask);
        cv::waitKey(3);
#endif

        cv::cvtColor(img, img, cv::COLOR_HSV2BGR);

        return mask;
    }

    ros::NodeHandle nh_;
    ros::Subscriber rgb_image_subscriber_;
    ros::Subscriber depth_image_subscriber_;
    ros::Subscriber camera_info_subscriber_;
    ros::Publisher odom_pub_;
    ros::Publisher position_publisher_;
    CameraInstrinsics camera_intrinsics_;
    cv_bridge::CvImagePtr cv_ptr_bgr_;
    cv_bridge::CvImagePtr cv_ptr_depth_;
    std::vector<ColorRange> color_ranges_;
    tf::TransformListener listener_;
    tf::Transform camera_to_world_transform_;
    std::string rgb_image_topic_;
    std::string depth_image_topic_;
    std::string camera_info_topic_;
    std::string camera_frame_id_;
    std::string ns_;
    cv::Point3d previous_point_;
    ros::Time last_time_;
    int gaussian_blur_kernel_size_;
    bool do_gaussian_blur_;
    bool do_histogram_equalization_;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "ballfinder");

    ros::NodeHandle nh("~");
    BallFinder bf(nh);

    ros::spin();

    return 0;
}