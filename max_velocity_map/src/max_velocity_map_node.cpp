#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <amcl/map/map.h>
#include <nav_core/base_local_planner.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <algorithm>

#include <pluginlib/class_list_macros.h>

class MaxVelocityMap // : public nav_core::BaseLocalPlanner
{
  public:
    MaxVelocityMap();
    ~MaxVelocityMap();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber map_sub_;
    map_t* map_;
    dynamic_reconfigure::ReconfigureRequest srv_req_;
    dynamic_reconfigure::ReconfigureResponse srv_resp_;
    dynamic_reconfigure::DoubleParameter double_param_;
    dynamic_reconfigure::Config conf_;
    float max_vel_x_initial_;
    float min_vel_x_initial_;
    // float max_vel_theta_initial_;
    // float min_vel_theta_initial_;
    void freeMapDependentMemory();
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    map_t* convertMap (const nav_msgs::OccupancyGrid& map_msg);
};

//register this planner as a BaseLocalPlanner plugin
// PLUGINLIB_EXPORT_CLASS(max_velocity_map::MaxVelocityMap, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(MaxVelocityMap, nav_core::BaseLocalPlanner)

int main(int argc, char **argv)
{

  ros::init(argc, argv, "max_velocity_map");
  new MaxVelocityMap();
  ros::spin();
  return(0);
}

MaxVelocityMap::MaxVelocityMap()
{
  nh_.getParam("/move_base/TrajectoryPlannerROS/max_vel_x", max_vel_x_initial_);
  nh_.getParam("/move_base/TrajectoryPlannerROS/min_vel_x", min_vel_x_initial_);
  // nh_.getParam("/move_base/TrajectoryPlannerROS/max_vel_theta", max_vel_theta_initial_);
  ROS_INFO("max_vel_x_initial_: %f", max_vel_x_initial_);
  ROS_INFO("min_vel_x_initial_: %f", min_vel_x_initial_);
  // ROS_INFO("max_vel_theta_initial_: %f", max_vel_theta_initial_);
  amcl_sub_ = nh_.subscribe("amcl_pose", 100, &MaxVelocityMap::amclCallback, this);
  map_sub_ = nh_.subscribe("map", 1, &MaxVelocityMap::mapReceived, this);
}

MaxVelocityMap::~MaxVelocityMap()
{
  freeMapDependentMemory();
  // restore max_vel parameters
  conf_.doubles.clear();
  double_param_.name = "max_vel_x";
  double_param_.value = max_vel_x_initial_;
  conf_.doubles.push_back(double_param_);
  // double_param_.name = "max_vel_theta";
  // double_param_.value = max_vel_theta_initial_;
  // conf_.doubles.push_back(double_param_);
  srv_req_.config = conf_;
  ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req_, srv_resp_);
}

void MaxVelocityMap::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
}

void MaxVelocityMap::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  float pos_x = msg->pose.pose.position.x;
  float pos_y = msg->pose.pose.position.y;
  float pos_z = msg->pose.pose.position.z;
  // ROS_INFO("pos_x: %f, pos_y: %f", pos_x, pos_y);
  // ROS_INFO("Current robot position [%f, %f, %f]", pos_x, pos_y, pos_z);

  if( map_ != NULL ) {
    // get map pixel value of current position
    int index_x = (int)((pos_x - map_->origin_x) / map_->scale);
    int index_y = (int)((pos_y - map_->origin_y) / map_->scale);
    float max_vel_ratio = map_->cells[index_x + index_y * map_->size_x].occ_state / 255.0;
    // ROS_INFO("index_x: %d, index_y: %d", index_x, index_y);
    // ROS_INFO("pixel value: %d", map_->cells[index_x + index_y * map_->size_x].occ_state);
    // ROS_INFO("max_vel_ratio: %f", max_vel_ratio);
    // set max_vel parameters
    conf_.doubles.clear();
    double_param_.name = "max_vel_x";
    double_param_.value = std::max(max_vel_x_initial_ * max_vel_ratio, min_vel_x_initial_); // avoid too slow movement
    ROS_INFO("max_vel_x: %f", double_param_.value);
    conf_.doubles.push_back(double_param_);
    // double_param_.name = "max_vel_theta";
    // double_param_.value = std::max(max_vel_theta_initial_ * max_vel_ratio, min_vel_theta_initial_); // avoid too slow movement
    // ROS_INFO("max_vel_theta: %f", double_param_.value);
    // conf_.doubles.push_back(double_param_);
    srv_req_.config = conf_;
    ros::service::call("/move_base/TrajectoryPlannerROS/set_parameters", srv_req_, srv_resp_);
  }
}

void MaxVelocityMap::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  handleMapMessage( *msg );
}

void MaxVelocityMap::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();

  map_ = convertMap(msg);
}

map_t*
MaxVelocityMap::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  // map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  // map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  map->origin_x = map_msg.info.origin.position.x;
  map->origin_y = map_msg.info.origin.position.y;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    map->cells[i].occ_state = (unsigned char)map_msg.data[i]; // black: 0, white: 255
  }

  return map;
}
