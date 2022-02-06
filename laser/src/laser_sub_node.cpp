#include <laser/laser_sub.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_sub_node");
    ros::NodeHandle nh;
    Laser pub(nh);
    ros::spin();
    return 0;
}