#include "dijkstra_planning/dijkstra_planner.hpp"

namespace dijkstra_planning
{
DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_node"){
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    this->declare_parameter("map_topic", "grid_prob_map");
    this->declare_parameter("rect_length", 2.0);
    this->declare_parameter("rect_half_width", 0.2);
    this->declare_parameter("occ_thresh", 40);
    this->declare_parameter("stride", 1);

    this->get_parameter("map_topic", map_topic_);
    this->get_parameter("rect_length", rect_length_);
    this->get_parameter("rect_half_width", rect_half_width_);
    this->get_parameter("occ_thresh", occ_thresh_);
    this->get_parameter("stride", stride_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, qos, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));
    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "clicked_point", qos, std::bind(&DijkstraPlanner::pointCallback, this, std::placeholders::_1));
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "dijkstra/visited_map", qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "dijkstra/path", qos);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "dijkstra/obstacles", rclcpp::QoS(rclcpp::KeepLast(1)));
    front_obs_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "dijkstra/front_obstacles", rclcpp::QoS(1));
    
    frequency_ = this->declare_parameter("frequency", 10.0);
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / frequency_));
    front_scan_timer_ = this->create_wall_timer(
        period, std::bind(&DijkstraPlanner::dectFrontRectTimerCb, this));
}

void DijkstraPlanner::dectFrontRectTimerCb(){
    // 地图或 TF 未就绪就先不跑
    if (!map_ || map_->data.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Map not ready, skip front-rect scan.");
        return;
    }

    std::vector<geometry_msgs::msg::Point> obstacles;
    geometry_msgs::msg::Point picked;
    Side side;
    if (findObstaclesInFrontRect(rect_length_, rect_half_width_, obstacles, occ_thresh_, stride_)) {
        RCLCPP_INFO(get_logger(), "Found %zu obstacles in front rectangle.", obstacles.size());
        // for(auto const & obstacle : obstacles){ // obstacles 里就是所有命中的障碍点（map 坐标）
        //     RCLCPP_INFO(this->get_logger(), "obstacle : x= %f, y= %f", obstacle.x, obstacle.y);
        // }
        visualizeFrontRectObstacles(obstacles, rect_length_, rect_half_width_);

        if (pickFrontEdgeObstacle(obstacles, picked, side)) {
            const char* s = (side==Side::LEFT ? "LEFT" : side==Side::RIGHT ? "RIGHT" :
                            side==Side::MIXED ? "MIXED" : "UNKNOWN");
            RCLCPP_INFO(get_logger(), "Picked obstacle (%s): x=%.6f, y=%.6f", s, picked.x, picked.y);
            visualizePickedObstacle(picked, side);
        }else{
            side = Side::UNKNOWN;
            picked.x=0.0;
            picked.y=0.0;
            visualizePickedObstacle(picked, side);
        }
    } else {
        RCLCPP_INFO(get_logger(), "No obstacles in front rectangle.");
        visualizeFrontRectObstacles({}, rect_length_, rect_half_width_);
    }
}

void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map){
    map_ = map;
    if(map_){
        RCLCPP_INFO(this->get_logger(), "map received !");
        visited_map_.header.frame_id = map->header.frame_id;
        visited_map_.info = map->info;
        visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);
    }
    
}

void DijkstraPlanner::pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr point){
    if(!map_){
        RCLCPP_ERROR(this->get_logger(), "No map received!");
        return;
    }

    geometry_msgs::msg::PointStamped goal_in_map;
    try {
        if (point->header.frame_id != map_->header.frame_id) {
        goal_in_map = tf_buffer_->transform(
            *point, map_->header.frame_id, tf2::durationFromSec(0.1));
        } else {
        goal_in_map = *point;
        }
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(),
                    "Transform Point to map failed: %s", ex.what());
        return;
    }

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goal_in_map.point.x;
    goal_pose.position.y = goal_in_map.point.y;
    goal_pose.position.z = 0.0;
    goal_pose.orientation.w = 1.0;  // 单位四元数

    // 刷新地图
    visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);
    geometry_msgs::msg::TransformStamped map_to_base_tf;
    try{// target_frame目标  source_frame源  lastest_frame最新一帧  
        map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
    }catch(const tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "Could not transform from map to base_footprint");
        return;
    }

    geometry_msgs::msg::Pose map_to_base_pose;
    map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
    map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
    map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

    auto path = plan(map_to_base_pose, goal_pose);
    if(!path.poses.empty()){
        RCLCPP_INFO(this->get_logger(), "Shortest path found!");
        path_pub_->publish(path);
    }else{
        RCLCPP_WARN(this->get_logger(), "No path found to the goal");
    }
}

nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose & start, const geometry_msgs::msg::Pose & goal){
    // 1、探索方向
    std::vector<std::pair<int, int>> explore_directions={
        // {-1, 0}, {1, 0}, {0, 1}, {0, -1}
        {1, 0}, {0, 1}, {0, -1}
    };

    // 2、优先队列 <元素类型, 底层容器, 比较器>
    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodos;
    std::vector<GraphNode> visited_nodes;
    pending_nodos.push(worldToGrid(start));
    GraphNode active_node;
    visualization_msgs::msg::Marker m;

    while(!pending_nodos.empty() && rclcpp::ok()){
        active_node = pending_nodos.top();
        pending_nodos.pop();
        if(worldToGrid(goal)==active_node){
            break;
        }
        for(const auto & dir : explore_directions){
            GraphNode new_node = active_node + dir;
            // if(poseOnMap(new_node)){
            //     RCLCPP_INFO(this->get_logger(), "new_node cost: %d", map_->data.at(poseToCell(new_node)));
            // }
            
            if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && 
                poseOnMap(new_node) && map_->data.at(poseToCell(new_node))<=40 && map_->data.at(poseToCell(new_node))>-1){
                new_node.cost = active_node.cost + 1;
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodos.push(new_node);
                visited_nodes.push_back(new_node);
            }
            else if(poseOnMap(new_node) && map_->data.at(poseToCell(new_node))==-1){
                m.header.frame_id = map_->header.frame_id;
                m.header.stamp    = this->now();
                m.ns   = "dijkstra_obstacles";
                m.id   = 0;
                m.type = visualization_msgs::msg::Marker::POINTS;
                m.action = visualization_msgs::msg::Marker::ADD;
                // 点大小设为一个栅格的边长；POINTS 需要 scale.x & scale.y
                m.scale.x = map_->info.resolution;
                m.scale.y = map_->info.resolution;

                // 颜色：红色不透明
                m.color.r = 0.0f; m.color.g = 0.0f; m.color.b = 5.0f; m.color.a = 1.0f;
                geometry_msgs::msg::Pose pose = gridToWorld(new_node);
                geometry_msgs::msg::Point p;
                p.x = pose.position.x;
                p.y = pose.position.y;
                m.points.push_back(p);
            }
        }

        if(poseOnMap(active_node)){
            visited_map_.data.at(poseToCell(active_node)) = 10;
        }
        map_pub_->publish(visited_map_);
        obstacle_pub_->publish(m);
    }

    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = map_->header.frame_id;
    while(active_node.prev && rclcpp::ok()){
        geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
        geometry_msgs::msg::PoseStamped last_pose_stamped;
        last_pose_stamped.header.frame_id = map_->header.frame_id;
        last_pose_stamped.pose = last_pose;
        path.poses.push_back(last_pose_stamped);
        active_node = *active_node.prev;  //一直向前回溯
    }

    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{ // 计算栅格中 x、y坐标
    int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x)/map_->info.resolution);
    int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y)/map_->info.resolution);
    return GraphNode(grid_x, grid_y); 
}

bool DijkstraPlanner::poseOnMap(const GraphNode & node)
{ // 判断当前pose是否在地图中
    return node.x>=0 && node.x<static_cast<int>(map_->info.width) && 
        node.y>=0 && node.y<static_cast<int>(map_->info.height);
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
{ // 计算map数据的index
    return node.y * map_->info.width + node.x;
}

geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node)
{ // 栅格坐标转world坐标
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
    pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
    return pose;
}

geometry_msgs::msg::Point DijkstraPlanner::cellCenterToWorld(int gx, int gy) const {
  geometry_msgs::msg::Point p;
  const auto &info = map_->info;
  p.x = (gx + 0.5) * info.resolution + info.origin.position.x;
  p.y = (gy + 0.5) * info.resolution + info.origin.position.y;
  p.z = 0.0;
  return p;
}

double DijkstraPlanner::yawFromQuat(const geometry_msgs::msg::Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);   // yaw of (map->base) 旋转
}

bool DijkstraPlanner::findObstaclesInFrontRect(double length_m,
                                               double half_width_m,
                                               std::vector<geometry_msgs::msg::Point> &pts_in_map,
                                               int8_t occ_thresh,
                                               int stride)
{
    pts_in_map.clear();
    if (!map_ || map_->data.empty()) {
        RCLCPP_ERROR(get_logger(), "Map not ready.");
        return false;
    }

    // 1) TF：map -> base_footprint
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer_->lookupTransform(
            map_->header.frame_id, "base_footprint", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "TF map->base_footprint failed: %s", ex.what());
        return false;
    }

    const auto &T = map_to_base.transform;
    const double tx = T.translation.x;
    const double ty = T.translation.y;
    const double yaw = yawFromQuat(T.rotation);
    const double c = std::cos(yaw), s = std::sin(yaw);

    // 2) 将“基座坐标系下”的矩形 4 角转换到 map，用它的包围盒减少遍历范围
    auto base2map = [&](double xb, double yb) {
        geometry_msgs::msg::Point p;
        // v_map = t + R^T * v_base，R 为 map->base 旋转，因此 base->map 用 R^T
        p.x = tx + c * xb - s * yb;
        p.y = ty + s * xb + c * yb;
        p.z = 0.0;
        return p;
    };
    const auto p00 = base2map(0.0,         -half_width_m);
    const auto p01 = base2map(0.0,          half_width_m);
    const auto p10 = base2map(length_m,    -half_width_m);
    const auto p11 = base2map(length_m,     half_width_m);

    const auto &info = map_->info;
    const int W = static_cast<int>(info.width);
    const int H = static_cast<int>(info.height);
    const double res = info.resolution;

    auto world2grid = [&](double wx, double wy, int &gx, int &gy) {
        gx = static_cast<int>(std::floor((wx - info.origin.position.x) / res));
        gy = static_cast<int>(std::floor((wy - info.origin.position.y) / res));
    };

    int gx0, gy0, gx1, gy1, gx2, gy2, gx3, gy3;
    world2grid(p00.x, p00.y, gx0, gy0);
    world2grid(p01.x, p01.y, gx1, gy1);
    world2grid(p10.x, p10.y, gx2, gy2);
    world2grid(p11.x, p11.y, gx3, gy3);

    int gx_min = std::max(0, std::min(std::min(gx0, gx1), std::min(gx2, gx3)));
    int gx_max = std::min(W - 1, std::max(std::max(gx0, gx1), std::max(gx2, gx3)));
    int gy_min = std::max(0, std::min(std::min(gy0, gy1), std::min(gy2, gy3)));
    int gy_max = std::min(H - 1, std::max(std::max(gy0, gy1), std::max(gy2, gy3)));

    // 3) 扫描包围盒中的所有栅格：先判障碍，再判是否真在矩形内
    for (int gy = gy_min; gy <= gy_max; gy += std::max(1, stride)) {
        for (int gx = gx_min; gx <= gx_max; gx += std::max(1, stride)) {
            const int idx = gy * W + gx;
            const int8_t occ = map_->data[idx];
            if (occ <= occ_thresh) continue;              // 只取障碍（阈值可调）

            geometry_msgs::msg::Point pm = cellCenterToWorld(gx, gy);

            // map -> base，判断是否在矩形内（x∈[0,L] 且 |y|≤half_width）
            const double dx = pm.x - tx;
            const double dy = pm.y - ty;
            // v_base = R * v_map，R 为 map->base：
            const double xb =  c * dx + s * dy;
            const double yb =  -s * dx + c * dy;

            if (xb >= 0.0 && xb <= length_m && std::fabs(yb) <= half_width_m) {
                pts_in_map.push_back(pm);
            }
        }
    }

    return !pts_in_map.empty();
}

void DijkstraPlanner::visualizeFrontRectObstacles(const std::vector<geometry_msgs::msg::Point>& pts,
    double length_m, double half_width_m)
{
    if (!map_) return;
    visualization_msgs::msg::Marker m_pts;
    m_pts.header.frame_id = map_->header.frame_id;
    m_pts.header.stamp    = this->now();
    m_pts.ns = "front_rect_obstacles";
    m_pts.id = 0;
    m_pts.type = visualization_msgs::msg::Marker::POINTS;
    m_pts.action = visualization_msgs::msg::Marker::ADD;
    m_pts.scale.x = map_->info.resolution*2;       // 点大小
    m_pts.scale.y = map_->info.resolution*2;
    m_pts.color.r = 0.0f; m_pts.color.g = 0.0f; m_pts.color.b = 1.0f; m_pts.color.a = 1.0f;
    m_pts.points = pts;

    // 画矩形边框（LINE_STRIP，按 base 坐标转成 map 点）
    geometry_msgs::msg::TransformStamped map_to_base;
    try {
        map_to_base = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
    }catch(const tf2::TransformException &ex){
        RCLCPP_ERROR(this->get_logger(), "Could not transform from map to base_footprint");
        return;
    }

    visualization_msgs::msg::Marker m_box;
    m_box.header.frame_id = map_->header.frame_id;
    m_box.header.stamp = this->now();
    m_box.ns = "front_rect_box";
    m_box.id = 1;
    m_box.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m_box.action = visualization_msgs::msg::Marker::ADD;
    m_box.frame_locked = true;       // 关键：跟随 TF 更新
    m_box.scale.x = 0.05; // 线宽
    m_box.color.r = 0.0f; m_box.color.g = 1.0f; m_box.color.b = 6.6f; m_box.color.a = 1.0f;

    if (!map_to_base.header.frame_id.empty()) {
        const auto &T = map_to_base.transform;
        const double tx = T.translation.x, ty = T.translation.y;
        const double yaw = yawFromQuat(T.rotation);
        const double c = std::cos(yaw), s = std::sin(yaw);

        auto base2map = [&](double xb, double yb) {
        geometry_msgs::msg::Point p;
        p.x = tx + c * xb - s * yb;
        p.y = ty + s * xb + c * yb;
        p.z = 0.0;
        return p;
        };

        // 按顺时针：左后->右后->右前->左前->回到左后
        m_box.points.push_back(base2map(0.0,         -half_width_m));
        m_box.points.push_back(base2map(0.0,          half_width_m));
        m_box.points.push_back(base2map(length_m,     half_width_m));
        m_box.points.push_back(base2map(length_m,    -half_width_m));
        m_box.points.push_back(m_box.points.front());
    }

    front_obs_pub_->publish(m_pts);
    front_obs_pub_->publish(m_box);
}

bool DijkstraPlanner::pickFrontEdgeObstacle(
    const std::vector<geometry_msgs::msg::Point>& pts_in_map,
    geometry_msgs::msg::Point& chosen_in_map,
    Side& side)
{
  side = Side::UNKNOWN;
  if (pts_in_map.empty() || !map_) return false;

  int n_left = 0, n_right = 0;
  double best_left_y = std::numeric_limits<double>::infinity();   // 左侧取最小 y_base (越小越靠右)
  double best_right_y = -std::numeric_limits<double>::infinity(); // 右侧取最大 y_base (越大越靠左)
  geometry_msgs::msg::Point best_left_pt, best_right_pt;

  for (const auto& pm : pts_in_map) {
    auto tf = tf_buffer_->lookupTransform("base_footprint", map_->header.frame_id,
                                      tf2::TimePointZero);  // 最新
    const double tx = tf.transform.translation.x;
    const double ty = tf.transform.translation.y;
    const auto& q = tf.transform.rotation;
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(qq).getRPY(roll, pitch, yaw);
    const double c = std::cos(yaw), s = std::sin(yaw);

    // map -> base（用于左右判断）
    const double dx = pm.x - tx, dy = pm.y - ty;
    // base 坐标系下的左右：yb>0 左侧；yb<0 右侧
    const double yb = -s*dx + c*dy;  // y_base>0 左侧，<0 右侧

    if (yb >= 0.0) {
      ++n_left;
      if (yb < best_left_y) { best_left_y = yb; best_left_pt = pm; }
    } else {
      ++n_right;
      if (yb > best_right_y) { best_right_y = yb; best_right_pt = pm; }
    }
  }

  if (n_left > 0 && n_right == 0) {           // 全部在左侧
    side = Side::LEFT;
    chosen_in_map = best_left_pt;             // 取 yb 最小 -> 最“右”（最靠近中线）
    return true;
  }
  if (n_right > 0 && n_left == 0) {           // 全部在右侧
    side = Side::RIGHT;
    chosen_in_map = best_right_pt;            // 取 yb 最大 -> 最“左”（最靠近中线）
    return true;
  }

  // 左右混合：选离中线最近的 |yb| 最小
  if (n_left + n_right > 0) {
    side = Side::MIXED;
    // 两端候选：best_left_y >=0、best_right_y <=0
    if (n_left > 0 && n_right > 0) {
      if (best_left_y <= std::abs(best_right_y)) chosen_in_map = best_left_pt;
      else                                       chosen_in_map = best_right_pt;
    } else if (n_left > 0) {
      chosen_in_map = best_left_pt;
    } else {
      chosen_in_map = best_right_pt;
    }
    return true;
  }

  side = Side::UNKNOWN;
  return false;
}

void DijkstraPlanner::visualizePickedObstacle(const geometry_msgs::msg::Point& p_in_map,
                                              Side side)
{
  if (!map_) return;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = map_->header.frame_id;
  m.header.stamp = this->now();
  m.ns = "front_rect_obstacles";
  m.id = 2;                          // 与 POINTS(id=0)、矩形(id=1) 不冲突
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position = p_in_map;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.18; m.scale.y = 0.18; m.scale.z = 0.18;

  // 颜色：左侧蓝色，右侧黄色，混合白色
  if (side == Side::LEFT)      { m.color.r = 0.0f; m.color.g = 0.4f; m.color.b = 1.0f; m.color.a = 1.0f; }
  else if (side == Side::RIGHT){ m.color.r = 1.0f; m.color.g = 0.8f; m.color.b = 0.0f; m.color.a = 1.0f; }
  else                         { m.color.r = 1.0f; m.color.g = 1.0f; m.color.b = 1.0f; m.color.a = 1.0f; }

  front_obs_pub_->publish(m);
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto dijkstra_node = std::make_shared<ductcleanrob_planning::DijkstraPlanner>();
    rclcpp::spin(dijkstra_node);
    rclcpp::shutdown();
    return 0;
}