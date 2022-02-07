#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Laser{
private:
    ros::NodeHandle node_handle;
    ros::Subscriber laser_sub;

public:
    Laser(ros::NodeHandle& nh);

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& s);
}; 



