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
      typedef float _base_wheel_left_velocity_type;
      _base_wheel_left_velocity_type base_wheel_left_velocity;
      typedef float _base_wheel_right_velocity_type;
      _base_wheel_right_velocity_type base_wheel_right_velocity;
      typedef float _base_wheel_back_velocity_type;
      _base_wheel_back_velocity_type base_wheel_back_velocity;
      typedef float _kicker_effort_type;
      _kicker_effort_type kicker_effort;
      typedef bool _dribbler_active_type;
      _dribbler_active_type dribbler_active;

    HardwareCommandMsg():
      base_wheel_left_velocity(0),
      base_wheel_right_velocity(0),
      base_wheel_back_velocity(0),
      kicker_effort(0),
      dribbler_active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_wheel_left_velocity;
      u_base_wheel_left_velocity.real = this->base_wheel_left_velocity;
      *(outbuffer + offset + 0) = (u_base_wheel_left_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_wheel_left_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_wheel_left_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_wheel_left_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_wheel_left_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_right_velocity;
      u_base_wheel_right_velocity.real = this->base_wheel_right_velocity;
      *(outbuffer + offset + 0) = (u_base_wheel_right_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_wheel_right_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_wheel_right_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_wheel_right_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_wheel_right_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_back_velocity;
      u_base_wheel_back_velocity.real = this->base_wheel_back_velocity;
      *(outbuffer + offset + 0) = (u_base_wheel_back_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_wheel_back_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_wheel_back_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_wheel_back_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_wheel_back_velocity);
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
      union {
        bool real;
        uint8_t base;
      } u_dribbler_active;
      u_dribbler_active.real = this->dribbler_active;
      *(outbuffer + offset + 0) = (u_dribbler_active.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->dribbler_active);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_wheel_left_velocity;
      u_base_wheel_left_velocity.base = 0;
      u_base_wheel_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_wheel_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_wheel_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_wheel_left_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_wheel_left_velocity = u_base_wheel_left_velocity.real;
      offset += sizeof(this->base_wheel_left_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_right_velocity;
      u_base_wheel_right_velocity.base = 0;
      u_base_wheel_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_wheel_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_wheel_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_wheel_right_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_wheel_right_velocity = u_base_wheel_right_velocity.real;
      offset += sizeof(this->base_wheel_right_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_back_velocity;
      u_base_wheel_back_velocity.base = 0;
      u_base_wheel_back_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_wheel_back_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_wheel_back_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_wheel_back_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_wheel_back_velocity = u_base_wheel_back_velocity.real;
      offset += sizeof(this->base_wheel_back_velocity);
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
      union {
        bool real;
        uint8_t base;
      } u_dribbler_active;
      u_dribbler_active.base = 0;
      u_dribbler_active.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->dribbler_active = u_dribbler_active.real;
      offset += sizeof(this->dribbler_active);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/HardwareCommandMsg"; };
    const char * getMD5(){ return "6ab45c676bc586ec47b7fe24a9bbfad9"; };

  };

}
#endif