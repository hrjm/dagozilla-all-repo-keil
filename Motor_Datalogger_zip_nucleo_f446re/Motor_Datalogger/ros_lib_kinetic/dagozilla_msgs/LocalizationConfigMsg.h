#ifndef _ROS_dagozilla_msgs_LocalizationConfigMsg_h
#define _ROS_dagozilla_msgs_LocalizationConfigMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msgs
{

  class LocalizationConfigMsg : public ros::Msg
  {
    public:
      typedef bool _use_compass_type;
      _use_compass_type use_compass;
      typedef bool _use_vision_type;
      _use_vision_type use_vision;

    LocalizationConfigMsg():
      use_compass(0),
      use_vision(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_use_compass;
      u_use_compass.real = this->use_compass;
      *(outbuffer + offset + 0) = (u_use_compass.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_compass);
      union {
        bool real;
        uint8_t base;
      } u_use_vision;
      u_use_vision.real = this->use_vision;
      *(outbuffer + offset + 0) = (u_use_vision.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->use_vision);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_use_compass;
      u_use_compass.base = 0;
      u_use_compass.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_compass = u_use_compass.real;
      offset += sizeof(this->use_compass);
      union {
        bool real;
        uint8_t base;
      } u_use_vision;
      u_use_vision.base = 0;
      u_use_vision.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->use_vision = u_use_vision.real;
      offset += sizeof(this->use_vision);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/LocalizationConfigMsg"; };
    const char * getMD5(){ return "4239db2834f07274ab49235e74faa3f2"; };

  };

}
#endif