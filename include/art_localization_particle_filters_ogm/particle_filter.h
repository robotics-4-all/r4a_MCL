#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <art_localization_particle_filters_ogm/particle.h>
#include <art_localization_particle_filters_ogm/robot_perception.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <art_localization_particle_filters_ogm/particleInitSrv.h>


class ParticleFilter {
  private:
    ros::NodeHandle _n;
    ros::ServiceServer _particle_initialization_service;
    ros::Publisher _visualization_pub;
    ros::Subscriber _velocity_sub;
    ros::Subscriber _odometry_sub;
    ros::Time _current_time;
    ros::Time _previous_time;
    ros::Timer _timer;
    ros::Duration _dt;
    std::string _velocity_topic;
    std::string _odometry_topic;
    RobotPerception robot_percept;
    int _particles_number;
    int _duration;
    double _noise_param1;
    double _noise_param2;
    float _current_linear;
    float _current_angular;
    float _previous_linear;
    float _previous_angular;
    std::vector<Particle> _particles; 
    bool _visualization_enabled;
    bool _particles_initialized;
    bool _flag;
    bool _motion_flag;
    int _subs_step;
    double _strict;

    bool _start_from_point;
    int _start_x;
    int _start_y;
    float _start_theta;
      
  public:
    ParticleFilter();
    bool particlesInit ( 
      art_localization_particle_filters_ogm::particleInitSrv::Request& req,
      art_localization_particle_filters_ogm::particleInitSrv::Response& res
      );
    void visualize(float resolution);
    void particlesCallback(const ros::TimerEvent& event);
    void velocityCallback(geometry_msgs::Twist twist);
    void odometryCallback(nav_msgs::Odometry odometry);
    void resample();
};

#endif
