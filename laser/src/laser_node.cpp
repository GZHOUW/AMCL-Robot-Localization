#include <laser/Laser.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "laser_node");
    ros::NodeHandle nh;
    Laser pub(nh);
    ros::spin();
    return 0;
}