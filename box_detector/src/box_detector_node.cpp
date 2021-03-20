#include <box_detector/box_detector.hpp>

using namespace interiit21::box_detection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "box_detector_node");
    ros::NodeHandle nh;

    BoxDetectorNode detect;

    detect.init(nh);

    ros::Rate loop_rate(30);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
