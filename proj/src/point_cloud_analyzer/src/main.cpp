#include "PointCloudAnalyzer.hpp"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "point_clound_analyzer");
    PointCloudAnalyzer point_cloud_analyzer;
    point_cloud_analyzer.start();
    return 0;
}
