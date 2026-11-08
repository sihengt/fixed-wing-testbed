// Publish the planner's no-fly circles as an octomap, so cf_drone's jps3d node plans in the same world.
// One layer of voxels at the flight altitude (NED z). Republished every second: jps3d's subscription is
// volatile, so a late start would otherwise never get the map.
#include <cmath>
#include <vector>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("world_octomap");
    const auto obstacles =
        node->declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});  // x, y, r, ...
    const double z = node->declare_parameter<double>("z", -60.0);
    const double resolution = node->declare_parameter<double>("resolution", 2.0);
    const auto frame_id = node->declare_parameter<std::string>("frame_id", "px4_local_ned");

    octomap::OcTree tree(resolution);
    for (size_t i = 0; i + 2 < obstacles.size(); i += 3)
    {
        const double ox = obstacles[i], oy = obstacles[i + 1], r = obstacles[i + 2];
        for (double x = ox - r; x <= ox + r; x += resolution / 2)
            for (double y = oy - r; y <= oy + r; y += resolution / 2)
                if (std::hypot(x - ox, y - oy) <= r)
                    tree.updateNode(octomap::point3d(x, y, z), true);
    }
    tree.updateInnerOccupancy();

    octomap_msgs::msg::Octomap msg;
    octomap_msgs::fullMapToMsg(tree, msg);
    msg.header.frame_id = frame_id;
    auto pub = node->create_publisher<octomap_msgs::msg::Octomap>("/octomap_full", 10);
    auto timer = node->create_wall_timer(std::chrono::seconds(1), [&]() {
        msg.header.stamp = node->now();
        pub->publish(msg);
    });
    RCLCPP_INFO(node->get_logger(), "Publishing %zu no-fly circles as %zu occupied voxels (%.1f m)",
                obstacles.size() / 3, tree.getNumLeafNodes(), resolution);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
