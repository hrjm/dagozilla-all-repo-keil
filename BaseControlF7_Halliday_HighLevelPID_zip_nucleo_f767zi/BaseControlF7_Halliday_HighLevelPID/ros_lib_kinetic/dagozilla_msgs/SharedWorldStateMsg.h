#ifndef _ROS_dagozilla_msgs_SharedWorldStateMsg_h
#define _ROS_dagozilla_msgs_SharedWorldStateMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "dagozilla_msgs/KineticBodyMsg.h"

namespace dagozilla_msgs
{

  class SharedWorldStateMsg : public ros::Msg
  {
    public:
      typedef const char* _robot_id_type;
      _robot_id_type robot_id;
      typedef ros::Time _timestamp_type;
      _timestamp_type timestamp;
      typedef bool _ball_in_possession_type;
      _ball_in_possession_type ball_in_possession;
      typedef bool _ball_in_sight_type;
      _ball_in_sight_type ball_in_sight;
      typedef dagozilla_msgs::KineticBodyMsg _my_kinematics_type;
      _my_kinematics_type my_kinematics;
      typedef dagozilla_msgs::KineticBodyMsg _ball_kinematics_type;
      _ball_kinematics_type ball_kinematics;

    SharedWorldStateMsg():
      robot_id(""),
      timestamp(),
      ball_in_possession(0),
      ball_in_sight(0),
      my_kinematics(),
      ball_kinematics()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_robot_id = strlen(this->robot_id);
      varToArr(outbuffer + offset, length_robot_id);
      offset += 4;
      memcpy(outbuffer + offset, this->robot_id, length_robot_id);
      offset += length_robot_id;
      *(outbuffer + offset + 0) = (this->timestamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.sec);
      *(outbuffer + offset + 0) = (this->timestamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestamp.nsec);
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_robot_id;
      arrToVar(length_robot_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_robot_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_robot_id-1]=0;
      this->robot_id = (char *)(inbuffer + offset-1);
      offset += length_robot_id;
      this->timestamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.sec);
      this->timestamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestamp.nsec);
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
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/SharedWorldStateMsg"; };
    const char * getMD5(){ return "81fcd42a298660f2185cc70a38a8a580"; };

  };

}
#endif