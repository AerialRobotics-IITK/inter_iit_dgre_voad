#include <lander/lander.hpp>

using namespace interiit21::lander;

int main(int argc, char** argv) {
    ros::init(argc, argv, "lander_node");
    ros::NodeHandle nh;

    LanderNode land;

    land.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        land.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
