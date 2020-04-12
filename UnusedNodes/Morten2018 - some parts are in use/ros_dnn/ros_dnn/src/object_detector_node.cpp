#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(),
            "ros_dnn/ObjectDetectorNodelet",
            remap, nargv);

    ros::spin();

    return 0;
}
