#ifndef _ROS_dagozilla_msg_LedMsg_h
#define _ROS_dagozilla_msg_LedMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msg
{

  class LedMsg : public ros::Msg
  {
    public:
      typedef bool _red_type;
      _red_type red;
      typedef bool _green_type;
      _green_type green;
      typedef bool _blue_type;
      _blue_type blue;

    LedMsg():
      red(0),
      green(0),
      blue(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_red;
      u_red.real = this->red;
      *(outbuffer + offset + 0) = (u_red.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->red);
      union {
        bool real;
        uint8_t base;
      } u_green;
      u_green.real = this->green;
      *(outbuffer + offset + 0) = (u_green.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->green);
      union {
        bool real;
        uint8_t base;
      } u_blue;
      u_blue.real = this->blue;
      *(outbuffer + offset + 0) = (u_blue.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blue);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_red;
      u_red.base = 0;
      u_red.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->red = u_red.real;
      offset += sizeof(this->red);
      union {
        bool real;
        uint8_t base;
      } u_green;
      u_green.base = 0;
      u_green.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->green = u_green.real;
      offset += sizeof(this->green);
      union {
        bool real;
        uint8_t base;
      } u_blue;
      u_blue.base = 0;
      u_blue.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blue = u_blue.real;
      offset += sizeof(this->blue);
     return offset;
    }

    const char * getType(){ return "dagozilla_msg/LedMsg"; };
    const char * getMD5(){ return "13bca4c90aa92e68023254cf5e82c226"; };

  };

}
#endif