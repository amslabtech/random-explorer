#include <random>
#include <random_explorer/random_explorer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

namespace random_explorer {
RandomExplorer::RandomExplorer(Param param)
    : param_(param)
    , mt_(std::mt19937(std::random_device()()))
    , direction_groups_(param_)
    , has_localmap_(false)
    , has_odom_(false)
    , prev_angle_(0.0)
{
    sub_localmap_ = nh_.subscribe("/localmap", 1, &RandomExplorer::localmap_callback_,
                                  this, ros::TransportHints().reliable().tcpNoDelay());
    sub_odometry_ = nh_.subscribe("/odom", 1, &RandomExplorer::odometry_callback_,
                                  this, ros::TransportHints().reliable().tcpNoDelay());
    pub_local_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 2);

    local_goal_.pose.orientation.w = 1.0;
    local_goal_.header.frame_id = "base_link";
}

void RandomExplorer::localmap_callback_(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (msg->data.size() == 0) {
        return;
    }
    if (!has_localmap_) {
        has_localmap_ = true;
    }

    localmap_msg_ = *msg;
}

void RandomExplorer::odometry_callback_(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!has_odom_) {
        prev_odom_ = *msg;
        has_odom_ = true;
    }

    tf2::Transform trans_cur_odom;
    tf2::Transform trans_prev_odom;
    tf2::Transform trans_local_goal;

    tf2::fromMsg(msg->pose.pose, trans_cur_odom);
    tf2::fromMsg(prev_odom_.pose.pose, trans_prev_odom);
    tf2::fromMsg(local_goal_.pose, trans_local_goal);

    tf2::Transform trans_odom_diff = trans_prev_odom.inverse() * trans_cur_odom;
    tf2::Transform new_local_goal = trans_odom_diff.inverse() * trans_local_goal;
    tf2::toMsg(new_local_goal, local_goal_.pose);

    prev_odom_ = *msg;
}

bool RandomExplorer::is_close_angle(const double a, const double b, const double torelance)
{
    const double diff = std::abs(a - b);
    return diff < torelance;
}

void RandomExplorer::get_localmap(void)
{
    localmap_ = localmap_msg_;
}

bool RandomExplorer::reached_goal(void)
{
    const double dist_to_goal = std::hypot(
        local_goal_.pose.position.x, local_goal_.pose.position.y);

    return dist_to_goal < param_.goal_torelance;
}

bool RandomExplorer::is_valid_index(const int px, const int py)
{
    const bool is_valid_px = (0 <= px && px < static_cast<int>(localmap_.info.width));
    const bool is_valid_py = (0 <= py && py < static_cast<int>(localmap_.info.height));

    if (is_valid_px && is_valid_py) {
        return true;
    } else {
        return false;
    }
}

bool RandomExplorer::is_free_space(double x, double y)
{
    const int px = static_cast<int>(x / localmap_.info.resolution + localmap_.info.width / 2.0);
    const int py = static_cast<int>(y / localmap_.info.resolution + localmap_.info.height / 2.0);
    if (!is_valid_index(px, py)) {
        return false;
    }

    return (localmap_.data[px + py * localmap_.info.width] == 0);
}

bool RandomExplorer::should_search_free_space()
{
    return (
        reached_goal()
        || !is_free_space(local_goal_.pose.position.x, local_goal_.pose.position.y));
}

int RandomExplorer::count_free_grid(const double angle)
{
    int count = 0;
    int px_pre = -1;
    int py_pre = -1;

    const int map_range = std::min(
        static_cast<int>(localmap_.info.width), static_cast<int>(localmap_.info.height));
    for (int r = 0; r <= map_range; r++) {
        const int px = r * std::cos(angle) + localmap_.info.width / 2.0;
        const int py = r * std::sin(angle) + localmap_.info.height / 2.0;

        if (!is_valid_index(px, py)) {
            continue;
        }
        if (px == px_pre && py == py_pre) {
            continue;
        }

        int index = px + py * localmap_.info.width;
        if (localmap_.data[index] == 0) {
            count++;
        }
        px_pre = px;
        py_pre = py;
    }

    return count;
}

void RandomExplorer::search_free_spaces(void)
{
    for (int i = 0; i < param_.n_direction_groups; i++) {
        int free_grid_count = 0;
        for (const auto angle : direction_groups_.get_angles_at(i)) {
            free_grid_count += count_free_grid(angle);
        }

        direction_groups_.set_free_grid_count_at(i, free_grid_count);
    }
}

void RandomExplorer::decide_next_goal(void)
{
    direction_groups_.sort_by_free_grid_count();
    std::vector<int> counts;
    for (int i = 0; i < param_.n_direction_groups; i++) {
        counts.push_back(direction_groups_.get_free_grid_count_at(i));
    }

    std::discrete_distribution<int> discrete_dist(counts.begin(), counts.end());
    std::uniform_real_distribution<double> uniform_dist(
            param_.goal_torelance, localmap_.info.width * localmap_.info.resolution / 2.0);

    const double goal_distance = uniform_dist(mt_);
    const double drawn_angle = direction_groups_.get_center_angle_at(discrete_dist(mt_));
    const double x = goal_distance * std::cos(drawn_angle);
    const double y = goal_distance * std::sin(drawn_angle);
    if (!is_free_space(x, y)) {
        decide_next_goal();
    }

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, drawn_angle);
    local_goal_.pose.position.x = x;
    local_goal_.pose.position.y = y;
    local_goal_.pose.orientation = tf2::toMsg(quat);
}

void RandomExplorer::process(void)
{
    ros::Rate loop_rate(param_.hz);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

        local_goal_.header.stamp = ros::Time::now();
        pub_local_goal_.publish(local_goal_);
        get_localmap();
        if (!(has_localmap_ && has_odom_)) {
            continue;
        }
        if (!should_search_free_space()) {
            continue;
        }
        search_free_spaces();
        decide_next_goal();
    }
}
} // namespace random_explorer
