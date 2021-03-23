#include <active_planner/local_planner.hpp>

using namespace local_planner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_planner");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    FLAGS_alsologtostderr = true;

    LocalPlanner planner(nh, nh_private);

    ros::Rate loop_rate(40);

    while (ros::ok() && !planner.readyForExit()) {
        ros::spinOnce();
        if(planner.isActive()){
            planner.run();
        }
        loop_rate.sleep();
    }
}
      