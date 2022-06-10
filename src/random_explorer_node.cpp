#include <random_explorer/random_explorer.h>

int main(int argc, char** argv)
{
    random_explorer::Param param;

    ros::init(argc, argv, "random_explorer");
    ros::param::param<int>("~hz", param.hz, 10);
    ros::param::param<int>("~n_angle_samples", param.n_angle_samples, 360);
    ros::param::param<int>("~n_direction_groups", param.n_direction_groups, 8);
    ros::param::param<double>("~goal_torelance", param.goal_torelance, 3.0);

    random_explorer::RandomExplorer random_explorer(param);
    random_explorer.process();

    return 0;
}
