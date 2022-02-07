#include <laser/Laser.hpp>

Laser::Laser(ros::NodeHandle& nh){
    node_handle = nh;
    // call callback function in constructor
    laser_sub = nh.subscribe("base_scan", 1000, &Laser::laser_callback, this);
}

void Laser::laser_callback(const sensor_msgs::LaserScan::ConstPtr& s){
    // parse the received laser scan data
    float a_min = s->angle_min; // start angle of the scan

    float increment = s-> angle_increment;
    std::vector<float> ranges = s->ranges;

    ROS_INFO_STREAM(ranges[0]);
    float detect_r_max, detect_a_min, detect_a_max;
    float sum = 0;
    int size = 0;

    for (int i = 0; i < ranges.size(); i++){
        float r = ranges[i];
        float a = a_min + i * increment;

        // discard ranges that are out of bound
        if (r < s->range_min or r > s->range_max){
            continue;
        }

        if (r < detect_r_max and a > detect_a_min and a < detect_a_max){
            sum+=r;
            size++;
        }
    }

    float dist = sum/size;
    
   // ROS_INFO_STREAM(dist);
    
}