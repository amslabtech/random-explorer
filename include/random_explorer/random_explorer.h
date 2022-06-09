#ifndef __RANDOM_EXPLORER_H__
#define __RANDOM_EXPLORER_H__

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace random_explorer {
struct Param {
    int hz;
    int n_angle_samples;
    int n_direction_groups;
};

struct DirectionGroup {
    int id;
    double center_angle;
    int free_grid_count;
    std::vector<double> angles;
};

class DirectionGroups {
public:
    DirectionGroups(Param param);
    std::vector<double>& get_angles_at(const int id);
    void set_free_grid_count_at(const int id, const int free_space_count);
    int get_free_grid_count_at(const int id);
    double get_center_angle_at(const int id);
    void sort_by_free_grid_count(void);

private:
    const Param param_;
    std::vector<DirectionGroup> direction_groups_;
};

class RandomExplorer {
public:
    RandomExplorer(Param param);
    static int count_free_grid(const nav_msgs::OccupancyGrid& localmap, const double angle);
    static bool is_close_angle(const double a, const double b, const double torelance);
    void get_localmap(void);
    void search_free_spaces(void);
    void decide_next_goal(void);
    void process(void);

private:
    void localmap_callback_(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odometry_callback_(const nav_msgs::Odometry::ConstPtr& msg);

    const Param param_;
    ros::NodeHandle nh_;
    ros::Subscriber sub_localmap_;
    ros::Subscriber sub_odometry_;
    ros::Publisher pub_local_goal_;
    std::mt19937 mt_;

    nav_msgs::OccupancyGrid localmap_msg_;
    nav_msgs::OccupancyGrid localmap_;
    geometry_msgs::PoseStamped local_goal_;
    DirectionGroups direction_groups_;
    tf2::Quaternion prev_quat_;
    double prev_angle_;
};
} // namespace random_explorer

#endif // __RANDOM_EXPLORER_H__
