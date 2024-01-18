#include <ros/ros.h>
#include <string>

#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class StateFilter {
public:
    StateFilter(ros::NodeHandle& nh) : nh_(nh), ns_(ros::this_node::getNamespace()) {
        initROS();
        printParams();
    }
    
    void printParams() {
        ROS_WARN_STREAM("*** State Filter Parameters ***");
        ROS_WARN_STREAM("K: " << K_);
        ROS_WARN_STREAM("*** State Filter Parameters ***");
    }

    // callback with odometry type
    void stateCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Filter the state
        Vector6d state;
        state.head(3) = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        state.tail(3) = Eigen::Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

        Vector6d filtered_state = filterState(state);
        
        // Publish the filtered state
        nav_msgs::Odometry filtered_state_msg;
        filtered_state_msg.header = msg->header;
        filtered_state_msg.header.stamp = ros::Time::now();
        filtered_state_msg.pose.pose.position.x = filtered_state(0);
        filtered_state_msg.pose.pose.position.y = filtered_state(1);
        filtered_state_msg.pose.pose.position.z = filtered_state(2);
        filtered_state_msg.twist.twist.linear.x = filtered_state(3);
        filtered_state_msg.twist.twist.linear.y = filtered_state(4);
        filtered_state_msg.twist.twist.linear.z = filtered_state(5);
        state_pub_.publish(filtered_state_msg);

        // Publish the filtered position
        geometry_msgs::PointStamped position_msg;
        position_msg.header = msg->header;
        position_msg.header.stamp = ros::Time::now();
        position_msg.point.x = filtered_state(0);
        position_msg.point.y = filtered_state(1);
        position_msg.point.z = filtered_state(2);
        position_pub_.publish(position_msg);

    }

private:
    
    Vector6d filterState(const Vector6d& state) {
        if (is_first_) {
            state_prev_ = state;
            is_first_ = false;
        }
        auto filtered_state = K_ * state_prev_ + (1 - K_) * state;
        state_prev_ = filtered_state;
        return filtered_state;
    }

    void initROS() {
        
        // Fetch parameters
        if (!ros::param::has("K")) {
            ROS_ERROR("Parameter K not found");
            ros::shutdown();
        }
        ros::param::get("K", K_);

        // Initialize the subscribers and publishers
        sub_ = nh_.subscribe(ns_ + "/state", 10, &StateFilter::stateCallback, this);
        state_pub_ = nh_.advertise<nav_msgs::Odometry>(ns_ + "/state/filtered", 10);
        position_pub_ = nh_.advertise<geometry_msgs::PointStamped>(ns_ + "/position/filtered/", 10);
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher state_pub_;
    ros::Publisher position_pub_;
    Vector6d state_prev_;
    std::string ns_;
    double K_;
    bool is_first_;

};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "state_filter_node");
    ros::NodeHandle nh("~");
    
    // Create a StateFilter object
    StateFilter state_filter(nh);
    
    // Spin
    ros::spin();

    return 0;
}