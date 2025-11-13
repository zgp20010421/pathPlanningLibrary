#include <queue>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
using namespace std::chrono_literals;

namespace dijkstra_planning
{
struct GraphNode{
    int x;
    int y;
    int cost;
    std::shared_ptr<GraphNode> prev;

    GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0){

    }

    GraphNode() : GraphNode(0, 0){

    }

    // 重载运算符
    bool operator>(const GraphNode & other) const{
        return cost > other.cost;
    };

    bool operator==(const GraphNode & other) const{
        return x == other.x && y == other.y;
    };

    GraphNode operator+(std::pair<int, int> const & other) const{
        GraphNode res(x + other.first, y + other.second);
        return res;
    };
};


class DijkstraPlanner : public rclcpp::Node
{
public:
    DijkstraPlanner();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr front_obs_pub_;
    rclcpp::TimerBase::SharedPtr front_scan_timer_;

    std::string map_topic_;
    int stride_, occ_thresh_=40;
    double rect_length_=0., rect_half_width_=0., frequency_=0.;
    enum class Side { LEFT, RIGHT, MIXED, UNKNOWN };

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
    bool poseOnMap(const GraphNode & node);
    unsigned int poseToCell(const GraphNode & node);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
    geometry_msgs::msg::Point cellCenterToWorld(int gx, int gy) const;
    void visualizeFrontRectObstacles(const std::vector<geometry_msgs::msg::Point>& pts,
            double length_m, double half_width_m);
    double yawFromQuat(const geometry_msgs::msg::Quaternion &q);
    bool findObstaclesInFrontRect(double length_m,
                                    double half_width_m,
                                    std::vector<geometry_msgs::msg::Point> &pts_in_map,
                                    int8_t occ_thresh,
                                    int stride);
    void dectFrontRectTimerCb();  // 定时器回调
    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal);

    // 在矩形内的障碍点（map坐标）中，按“左/右侧规则”挑选一个点
    bool pickFrontEdgeObstacle(const std::vector<geometry_msgs::msg::Point>& pts_in_map,
                            geometry_msgs::msg::Point& chosen_in_map,
                            Side& side);

    // 可视化被选中的点（一个小球）
    void visualizePickedObstacle(const geometry_msgs::msg::Point& p_in_map,
                                Side side);

}; // class
}  // namespace
