#ifndef _ROS_dagozilla_msgs_HardwareCommandMsg_h
#define _ROS_dagozilla_msgs_HardwareCommandMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msgs
{

  class HardwareCommandMsg : public ros::Msg
  {
    public:
      typedef float _base_left_wheel_target_rate_type;
      _base_left_wheel_target_rate_type base_left_wheel_target_rate;
      typedef float _base_right_wheel_target_rate_type;
      _base_right_wheel_target_rate_type base_right_wheel_target_rate;
      typedef float _base_back_wheel_target_rate_type;
      _base_back_wheel_target_rate_type base_back_wheel_target_rate;
      typedef float _dribbler_left_wheel_target_rate_type;
      _dribbler_left_wheel_target_rate_type dribbler_left_wheel_target_rate;
      typedef float _dribbler_right_wheel_target_rate_type;
      _dribbler_right_wheel_target_rate_type dribbler_right_wheel_target_rate;
      typedef float _kicker_effort_type;
      _kicker_effort_type kicker_effort;

    HardwareCommandMsg():
      base_left_wheel_target_rate(0),
      base_right_wheel_target_rate(0),
      base_back_wheel_target_rate(0),
      dribbler_left_wheel_target_rate(0),
      dribbler_right_wheel_target_rate(0),
      kicker_effort(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_left_wheel_target_rate;
      u_base_left_wheel_target_rate.real = this->base_left_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_base_left_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_left_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_left_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_left_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_left_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_base_right_wheel_target_rate;
      u_base_right_wheel_target_rate.real = this->base_right_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_base_right_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_right_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_right_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_right_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_right_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_base_back_wheel_target_rate;
      u_base_back_wheel_target_rate.real = this->base_back_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_base_back_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_back_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_back_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_back_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_back_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_dribbler_left_wheel_target_rate;
      u_dribbler_left_wheel_target_rate.real = this->dribbler_left_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_dribbler_left_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_left_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_left_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_left_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_left_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_dribbler_right_wheel_target_rate;
      u_dribbler_right_wheel_target_rate.real = this->dribbler_right_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_dribbler_right_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_right_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_right_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_right_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_right_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_kicker_effort;
      u_kicker_effort.real = this->kicker_effort;
      *(outbuffer + offset + 0) = (u_kicker_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kicker_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kicker_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kicker_effort.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kicker_effort);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_left_wheel_target_rate;
      u_base_left_wheel_target_rate.base = 0;
      u_base_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_left_wheel_target_rate = u_base_left_wheel_target_rate.real;
      offset += sizeof(this->base_left_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_base_right_wheel_target_rate;
      u_base_right_wheel_target_rate.base = 0;
      u_base_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_right_wheel_target_rate = u_base_right_wheel_target_rate.real;
      offset += sizeof(this->base_right_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_base_back_wheel_target_rate;
      u_base_back_wheel_target_rate.base = 0;
      u_base_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_back_wheel_target_rate = u_base_back_wheel_target_rate.real;
      offset += sizeof(this->base_back_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_dribbler_left_wheel_target_rate;
      u_dribbler_left_wheel_target_rate.base = 0;
      u_dribbler_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_left_wheel_target_rate = u_dribbler_left_wheel_target_rate.real;
      offset += sizeof(this->dribbler_left_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_dribbler_right_wheel_target_rate;
      u_dribbler_right_wheel_target_rate.base = 0;
      u_dribbler_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_right_wheel_target_rate = u_dribbler_right_wheel_target_rate.real;
      offset += sizeof(this->dribbler_right_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_kicker_effort;
      u_kicker_effort.base = 0;
      u_kicker_effort.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kicker_effort.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kicker_effort.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kicker_effort.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kicker_effort = u_kicker_effort.real;
      offset += sizeof(this->kicker_effort);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/HardwareCommandMsg"; };
    const char * getMD5(){ return "f6568a19e93202c1a6141b8f58b6fec3"; };

  };

}
#endif
