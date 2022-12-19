#ifndef _ROS_dagozilla_msgs_LocalWorldStateMsg_h
#define _ROS_dagozilla_msgs_LocalWorldStateMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dagozilla_msgs/KineticBodyMsg.h"
#include "dagozilla_msgs/VectorMsg.h"

namespace dagozilla_msgs
{

  class LocalWorldStateMsg : public ros::Msg
  {
    public:
      typedef bool _ball_in_possession_type;
      _ball_in_possession_type ball_in_possession;
      typedef bool _ball_in_sight_type;
      _ball_in_sight_type ball_in_sight;
      typedef dagozilla_msgs::KineticBodyMsg _my_kinematics_type;
      _my_kinematics_type my_kinematics;
      typedef dagozilla_msgs::KineticBodyMsg _ball_kinematics_type;
      _ball_kinematics_type ball_kinematics;
      uint32_t obstacle_positions_length;
      typedef dagozilla_msgs::VectorMsg _obstacle_positions_type;
      _obstacle_positions_type st_obstacle_positions;
      _obstacle_positions_type * obstacle_positions;

    LocalWorldStateMsg():
      ball_in_possession(0),
      ball_in_sight(0),
      my_kinematics(),
      ball_kinematics(),
      obstacle_positions_length(0), obstacle_positions(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ball_in_possession;
      u_ball_in_possession.real = this->ball_in_possession;
      *(outbuffer + offset + 0) = (u_ball_in_possession.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ball_in_possession);
      union {
        bool real;
        uint8_t base;
      } u_ball_in_sight;
      u_ball_in_sight.real = this->ball_in_sight;
      *(outbuffer + offset + 0) = (u_ball_in_sight.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ball_in_sight);
      offset += this->my_kinematics.serialize(outbuffer + offset);
      offset += this->ball_kinematics.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->obstacle_positions_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->obstacle_positions_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->obstacle_positions_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->obstacle_positions_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->obstacle_positions_length);
      for( uint32_t i = 0; i < obstacle_positions_length; i++){
      offset += this->obstacle_positions[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_ball_in_possession;
      u_ball_in_possession.base = 0;
      u_ball_in_possession.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ball_in_possession = u_ball_in_possession.real;
      offset += sizeof(this->ball_in_possession);
      union {
        bool real;
        uint8_t base;
      } u_ball_in_sight;
      u_ball_in_sight.base = 0;
      u_ball_in_sight.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ball_in_sight = u_ball_in_sight.real;
      offset += sizeof(this->ball_in_sight);
      offset += this->my_kinematics.deserialize(inbuffer + offset);
      offset += this->ball_kinematics.deserialize(inbuffer + offset);
      uint32_t obstacle_positions_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      obstacle_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      obstacle_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      obstacle_positions_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->obstacle_positions_length);
      if(obstacle_positions_lengthT > obstacle_positions_length)
        this->obstacle_positions = (dagozilla_msgs::VectorMsg*)realloc(this->obstacle_positions, obstacle_positions_lengthT * sizeof(dagozilla_msgs::VectorMsg));
      obstacle_positions_length = obstacle_positions_lengthT;
      for( uint32_t i = 0; i < obstacle_positions_length; i++){
      offset += this->st_obstacle_positions.deserialize(inbuffer + offset);
        memcpy( &(this->obstacle_positions[i]), &(this->st_obstacle_positions), sizeof(dagozilla_msgs::VectorMsg));
      }
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/LocalWorldStateMsg"; };
    const char * getMD5(){ return "f91ddaf11425c3e001b0567095645847"; };

  };

}
#endif