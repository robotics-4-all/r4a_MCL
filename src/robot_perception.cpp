#include <art_localization_particle_filters_ogm/robot_perception.h>

/**
 * @brief Default Constructor. Gets map topic param and laser topic
 * param and subcribes to these topics.
 */
RobotPerception::RobotPerception () 
{
  map_initialized = false;

  //~ ROS_INFO_STREAM("RobotPerception Constructor");
  if(!_n.getParam("/map_topic", _map_topic_param))
  {
    ROS_ERROR("Map topic param does not exist");
  }	
  _map_sub = _n.subscribe(_map_topic_param, 1,
    &RobotPerception::mapCallback, this);

  if(!_n.getParam("/robot_laser_topic", _laser_topic_param))
  {
    ROS_ERROR("Laser topic param does not exist");
  }	
  _laser_sub = _n.subscribe(_laser_topic_param, 1,
    &RobotPerception::laserRangesCallback, this);

  if (!_n.getParam("/rfid_tags_topic", _rfid_tags_topic_param))
  {
    ROS_ERROR("Rfid_tags topic param does not exist");
  }
  _rfid_tags_sub = _n.subscribe(_rfid_tags_topic_param, 1,
    &RobotPerception::RfidTagsCallback, this);

  if (!_n.getParam("/rfid_reader_topic", _rfid_reader_topic_param))
  {
    ROS_ERROR("Rfid_reader topic param does not exist");
  }
  _rfid_reader_sub = _n.subscribe(_rfid_reader_topic_param, 1,
    &RobotPerception::RfidReaderCallback, this);
}

/**
 * @brief Gets the necessary info for the map (width, height, data)
 * @param occupancy_grid_msg [nav_msgs::OccupancyGrid] Message 
 * containing the map info
 */
void RobotPerception::mapCallback (
  nav_msgs::OccupancyGrid occupancy_grid_msg)
{	
  ROS_ERROR("Called!");  
  _map_width = occupancy_grid_msg.info.width;	
  _map_height = occupancy_grid_msg.info.height;
  _map_resolution = occupancy_grid_msg.info.resolution; 

  //~ ROS_INFO_STREAM ("RobotPerception:map_width ="<< " " << _map_width << 
  //~ " " << "RobotPerception:map_height ="<< " " << _map_height);

  _map_data = new int*[_map_width];
  for (unsigned int i = 0 ; i < _map_width ; i++)
  {
    _map_data[i] = new int[_map_height];
  }

  for (unsigned int j = 0 ; j < _map_height ; j++)
  {
    for (unsigned int i = 0 ; i < _map_width ; i++)
    {
      _map_data[i][j] =
        (int)occupancy_grid_msg.data[_map_width*j + i];
    }
  }

  map_initialized = true;  
}

void RobotPerception::RfidTagsCallback (stdr_msgs::RfidTagVector
  rfid_tag_msg)
{
  _rfid_tags = rfid_tag_msg.rfid_tags;

  std::ofstream data_file;
  data_file.open("/home/aspa/catkin_ws/src/thesis/localization_project/cfg/example.txt");
  for (unsigned int i = 0 ; i < _rfid_tags.size(); i++)
  {
    data_file << _rfid_tags[i].tag_id << "\t" <<
      _rfid_tags[i].pose.x << "\t" << _rfid_tags[i].pose.y << "\n"; 
  }
  data_file.close();

  std::string line;
  std::ifstream file ("/home/aspa/catkin_ws/src/thesis/localization_project/cfg/example.txt");
  if (file.is_open())
  {
    while (getline (file,line))
    {
      std::string id;
      float x, y;
      std::istringstream ss(line);
      ss >> id >> x >> y;
      _rfid_tags_id.push_back(id);
      _rfid_tags_x.push_back(x);
      _rfid_tags_y.push_back(y);
    }
    file.close();
  }	    
}

void RobotPerception::RfidReaderCallback (stdr_msgs::RfidSensorMeasurementMsg
  rfid_reader_msg)
{
  _rfid_pose.clear();
  _rfid_ids = rfid_reader_msg.rfid_tags_ids;
  _rfid_msgs = rfid_reader_msg.rfid_tags_msgs;
  RfidPose();
}

void RobotPerception::RfidPose()
{
  for (unsigned int i = 0 ; i < _rfid_ids.size() ; i++)
  {
    for (unsigned int j = 0 ; j < _rfid_tags_id.size() ; j++)
    {
      if (!_rfid_ids[i].compare(_rfid_tags_id[j]))
      {
        std::vector<float> temp;
        temp.push_back(_rfid_tags_x[j]);
        temp.push_back(_rfid_tags_y[j]);
        _rfid_pose.push_back(temp);
      }
    }
  }
  //~ for (unsigned int i = 0 ; i < _rfid_pose.size() ; i++)
  //~ {
  //~ ROS_INFO_STREAM (" i = " << i );
  //~ ROS_INFO_STREAM(" Pose x = " << _rfid_pose[i][0] << " y = " << _rfid_pose[i][1]);
  //~ }
}

/**
 * @brief Gets the laser ranges
 * @param laser_scan_msg [sensor_msgs::LaserScan] Message 
 * containing the laser ranges info
 */
void RobotPerception::laserRangesCallback(
  sensor_msgs::LaserScan laser_scan_msg) 
{
  _increment = laser_scan_msg.angle_increment;
  _angle_min = laser_scan_msg.angle_min;
  _laser_ranges = laser_scan_msg.ranges;
  _max_range = laser_scan_msg.range_max;
  for (unsigned int i = 0 ; i < _laser_ranges.size() ; i ++)
  {
    if (_laser_ranges[i] > _max_range)
      _laser_ranges[i] = _max_range;
  }
}

/**
 * @brief Returns the map width
 * @return unsigned int - Map width
 */
unsigned int RobotPerception::getMapWidth()
{
  return _map_width;
}

/**
 * @brief Returns the map height
 * @return unsigned int - Map height
 */	
unsigned int RobotPerception::getMapHeight()
{
  return _map_height;
}

float RobotPerception::getMapResolution()
{
  return _map_resolution;
} 

/**
 * @brief Returns the occupancy state of a map cell
 * @param i [int] Coordinate x of the map data array 
 * @param j [int] Coordinate y of the map data array 
 * @return int - Map cell occupancy state 
 */
int RobotPerception::getMapCell ( int i, int j ) 
{
  return _map_data[i][j];
}

/**
 * @brief Returns the map data
 * @return int** - Map occupancy data array
 */
int** RobotPerception::getMapData () 
{
  return _map_data;
}
/**
 * @brief Returns the laser ranges
 * @return float* - Laser ranges array
 */
std::vector<float> RobotPerception::getLaserRanges() 
{
  return _laser_ranges;
}

float RobotPerception::getRangeMax()
{
  return _max_range;
}

float RobotPerception::getAngleIncrement()
{
  return _increment;
}

float RobotPerception::getAngleMin()
{
  return _angle_min;
}

std::vector<std::string> RobotPerception::getRfidIds()
{
  return _rfid_ids;
}

std::vector<std::string> RobotPerception::getRfidMsgs()
{
  return _rfid_msgs;
}

std::vector<std::vector<float> > RobotPerception::getRfidPose()
{
  return _rfid_pose;
}

