#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <amcl/map/map.h>

class MaxVelMap
{
  public:
    MaxVelMap();
    ~MaxVelMap();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber map_sub_;
    map_t* map_;
    // geometry_msgs::Point pos_;
    void freeMapDependentMemory();
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    map_t* convertMap (const nav_msgs::OccupancyGrid& map_msg);
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "max_vel_map");
  // MaxVelMap::MaxVelMap max_vel_map;
  new MaxVelMap();
  // ros::Rate loop_rate(10);
  ros::spin();

  return(0);
}

MaxVelMap::MaxVelMap() // constructor
         // map_(NULL),
         // pos_(NULL)
         // latest_tf_valid_(false)
{
  amcl_sub_ = nh_.subscribe("amcl_pose", 100, &MaxVelMap::amclCallback, this);
  map_sub_ = nh_.subscribe("map", 1, &MaxVelMap::mapReceived, this);
}

MaxVelMap::~MaxVelMap()
{
  freeMapDependentMemory();
}

void MaxVelMap::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
}

void MaxVelMap::amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  float pos_x = msg->pose.pose.position.x;
  float pos_y = msg->pose.pose.position.y;
  float pos_z = msg->pose.pose.position.z;
  ROS_INFO("Current robot position [%f, %f, %f]", pos_x, pos_y, pos_z);

  if( map_ != NULL ) {
    int index_x = (int)((pos_x - map_->origin_x) / map_->scale + (map_->size_x / 2.0) - 0.5);
    int index_y = (int)((pos_y - map_->origin_y) / map_->scale + (map_->size_y / 2.0) - 0.5);

    ROS_INFO("index_x: %d, index_y: %d", index_x, index_y);

    int max_vel_ratio = map_->cells[index_x + index_y * map_->size_x].occ_state;

    ROS_INFO("max_vel_ratio: %d", max_vel_ratio);

    // TODO
    // change max(min)_vel_x(theta) based on max_vel_ratio
    // NOTE: USE DYNAMIC RECONFIGURE
  }
}

void MaxVelMap::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  handleMapMessage( *msg );
}

void MaxVelMap::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();

  map_ = convertMap(msg);
}

map_t*
MaxVelMap::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    // TODO set occupancy state which reflects max_velocity_map's color
    map->cells[i].occ_state = (unsigned char)map_msg.data[i]; // black: 0, white: 255
  }

  return map;
}
