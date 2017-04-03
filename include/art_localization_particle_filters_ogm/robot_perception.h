#ifndef ROBOT_PERCEPTION_H
#define ROBOT_PERCEPTION_H


#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <stdr_msgs/RfidSensorMeasurementMsg.h>
#include <stdr_msgs/RfidTagVector.h>
#include <stdr_msgs/RfidTag.h>
#include <fstream>
#include <sstream>
#include <iterator>

class RobotPerception
{
  private:
    ros::NodeHandle _n;
    std::string _map_topic_param;
    std::string _laser_topic_param;
    std::string _rfid_reader_topic_param;
    std::string _rfid_tags_topic_param;
    ros::Subscriber _map_sub;
    ros::Subscriber _laser_sub;
    ros::Subscriber _rfid_reader_sub;
    ros::Subscriber _rfid_tags_sub;
    unsigned int _map_width;
    unsigned int _map_height;
    float _map_resolution;
    int** _map_data; 
    std::vector<float> _laser_ranges;
    std::vector<std::string> _rfid_ids;
    std::vector<std::string> _rfid_msgs;
    std::vector<std::vector<float> > _rfid_pose;
    std::vector<stdr_msgs::RfidTag> _rfid_tags;
    std::vector<std::string> _rfid_tags_id;
    std::vector<float> _rfid_tags_x;
    std::vector<float> _rfid_tags_y;
    float _max_range;
    float _increment;
    float _angle_min;

  public:
    RobotPerception();
    void mapCallback(nav_msgs::OccupancyGrid occupancy_grid_msg);
    void laserRangesCallback(sensor_msgs::LaserScan laser_scan_msg);
    void RfidReaderCallback (stdr_msgs::RfidSensorMeasurementMsg rfid_reader_msg);
    void RfidTagsCallback (stdr_msgs::RfidTagVector rfid_tag_msg);
    void RfidPose();
    unsigned int getMapWidth();
    unsigned int getMapHeight();
    float getMapResolution();
    int** getMapData();
    int getMapCell (int i, int j);
    std::vector<float> getLaserRanges();
    float getRangeMax();
    float getAngleIncrement();
    float getAngleMin();
    std::vector<std::string> getRfidIds();
    std::vector<std::string> getRfidMsgs();
    std::vector<std::vector<float> > getRfidPose();

    bool map_initialized;
};

#endif
