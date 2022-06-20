#include <random_explorer/random_explorer.h>

namespace random_explorer {
DirectionGroups::DirectionGroups(Param param)
    : param_(param)
{
    const double angle_range = 2.0 * M_PI / param_.n_direction_groups;
    const double angle_step = 2.0 * M_PI / param_.n_angle_samples / param_.n_direction_groups;

    for (int i = 0; i < param_.n_direction_groups; i++) {
        const double center_angle = 2.0 * M_PI * i / param_.n_direction_groups;
        const double start_angle = center_angle - angle_range / 2.0;
        const double end_angle = center_angle + angle_range / 2.0;
        std::vector<double> angles;
        double cur_angle = start_angle;

        while (cur_angle < end_angle) {
            angles.push_back(cur_angle);
            cur_angle += angle_step;
        }

        direction_groups_.push_back({ i, center_angle, 0, angles });
    }
}

std::vector<double> DirectionGroups::get_angles_at(const int id)
{
    return direction_groups_[id].angles;
}

void DirectionGroups::set_free_grid_count_at(const int id, const int free_space_count)
{
    direction_groups_[id].free_grid_count = free_space_count;
}

int DirectionGroups::get_free_grid_count_at(const int id)
{
    return direction_groups_[id].free_grid_count;
}

double DirectionGroups::get_center_angle_at(const int id)
{
    return direction_groups_[id].center_angle;
}

void DirectionGroups::sort_by_free_grid_count(void)
{
    std::sort(direction_groups_.begin(), direction_groups_.end(),
        [](const DirectionGroup& a, const DirectionGroup& b) {
            return a.free_grid_count > b.free_grid_count;
        });
}
} // namespace random_explorer
