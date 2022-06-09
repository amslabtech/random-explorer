#include "tf2/utils.h"
#include <random_explorer/random_explorer.h>

namespace random_explorer {
RandomExplorer::RandomExplorer(Param param)
    : param_(param)
    , mt_(std::mt19937(std::random_device()()))
    , direction_groups_(param_)
    , prev_quat_(0, 0, 0, 1)
    , prev_angle_(0.0)
{
    sub_localmap_ = nh_.subscribe(
        "/localmap", 1, &RandomExplorer::localmap_callback_, this, ros::TransportHints().reliable().tcpNoDelay());
    sub_odometry_ = nh_.subscribe(
        "/odom", 1, &RandomExplorer::odometry_callback_, this, ros::TransportHints().reliable().tcpNoDelay());
    pub_local_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 2);
    local_goal_.header.frame_id = "base_link";
}

void RandomExplorer::localmap_callback_(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (msg->data.size() == 0) {
        return;
    }

    localmap_msg_ = *msg;
}

void RandomExplorer::odometry_callback_(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf2::Quaternion quat_odom;
    tf2::convert(msg->pose.pose.orientation, quat_odom);
    tf2::Quaternion quat_diff = quat_odom * prev_quat_.inverse();

    tf2::Quaternion quat_prev_angle;
    quat_prev_angle.setRPY(0, 0, prev_angle_);

    tf2::Quaternion quat_prev_angle_rotated = quat_prev_angle * quat_diff.inverse();
    prev_angle_ = tf2::getYaw(quat_prev_angle_rotated);
    prev_quat_ = quat_odom;
}

void RandomExplorer::get_localmap(void)
{
    localmap_ = localmap_msg_;
}

int RandomExplorer::count_free_grid(const nav_msgs::OccupancyGrid& localmap, const double angle)
{
    int count = 0;
    int px_pre = -1;
    int py_pre = -1;

    const int map_range = std::min(static_cast<int>(localmap.info.width), static_cast<int>(localmap.info.height));
    for (int r = 0; r <= map_range; r++) {
        const int px = r * std::cos(angle) + localmap.info.width / 2.0;
        const int py = r * std::sin(angle) + localmap.info.height / 2.0;
        const bool is_valid_px = 0 <= px && px < static_cast<int>(localmap.info.width);
        const bool is_valid_py = 0 <= py && py < static_cast<int>(localmap.info.height);

        if (!is_valid_px || !is_valid_py) {
            continue;
        }
        if (px == px_pre && py == py_pre) {
            continue;
        }

        int index = px + py * localmap.info.width;
        if (localmap.data[index] == 0) {
            count++;
        }
        px_pre = px;
        py_pre = py;
    }

    return count;
}

bool RandomExplorer::is_close_angle(const double a, const double b, const double torelance)
{
    const double diff = std::abs(a - b);
    return diff < torelance;
}

void RandomExplorer::search_free_spaces(void)
{
    for (int i = 0; i < param_.n_direction_groups; i++) {
        int free_grid_count = 0;
        for (const auto angle : direction_groups_.get_angles_at(i)) {
            free_grid_count += count_free_grid(localmap_, angle);
        }

        direction_groups_.set_free_grid_count_at(i, free_grid_count);
    }
}

void RandomExplorer::decide_next_goal(void)
{
    direction_groups_.sort_by_free_grid_count();

    if (is_close_angle(direction_groups_.get_center_angle_at(0), prev_angle_,
            2.0 * M_PI / param_.n_direction_groups)) {
        prev_angle_ = direction_groups_.get_center_angle_at(0);
        return;
    }

    std::vector<int> counts;
    for (int i = 0; i < param_.n_direction_groups; i++) {
        counts.push_back(direction_groups_.get_free_grid_count_at(i));
    }
    std::discrete_distribution<int> dist(counts.begin(), counts.end());

    const double range = std::min(localmap_.info.width, localmap_.info.height) * localmap_.info.resolution / 2;
    const double drawn_angle = direction_groups_.get_center_angle_at(dist(mt_));
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, drawn_angle);
    local_goal_.pose.position.x = range * std::cos(drawn_angle);
    local_goal_.pose.position.y = range * std::sin(drawn_angle);
    local_goal_.pose.orientation = tf2::toMsg(quat);

    prev_angle_ = drawn_angle;
}

void RandomExplorer::process(void)
{
    ros::Rate loop_rate(param_.hz);

    while (ros::ok()) {
        get_localmap();
        search_free_spaces();
        decide_next_goal();
        local_goal_.header.stamp = ros::Time::now();
        pub_local_goal_.publish(local_goal_);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
} // namespace random_explorer
