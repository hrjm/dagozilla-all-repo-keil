#ifndef _ROS_hardware_msg_HardwareCommand_h
#define _ROS_hardware_msg_HardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hardware_msg
{

  class HardwareCommand : public ros::Msg
  {
    public:
      typedef float _left_wheel_target_rate_type;
      _left_wheel_target_rate_type left_wheel_target_rate;
      typedef float _right_wheel_target_rate_type;
      _right_wheel_target_rate_type right_wheel_target_rate;
      typedef float _back_wheel_target_rate_type;
      _back_wheel_target_rate_type back_wheel_target_rate;

    HardwareCommand():
      left_wheel_target_rate(0),
      right_wheel_target_rate(0),
      back_wheel_target_rate(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_target_rate;
      u_left_wheel_target_rate.real = this->left_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_left_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_target_rate;
      u_right_wheel_target_rate.real = this->right_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_right_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_back_wheel_target_rate;
      u_back_wheel_target_rate.real = this->back_wheel_target_rate;
      *(outbuffer + offset + 0) = (u_back_wheel_target_rate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_back_wheel_target_rate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_back_wheel_target_rate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_back_wheel_target_rate.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->back_wheel_target_rate);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_target_rate;
      u_left_wheel_target_rate.base = 0;
      u_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_wheel_target_rate = u_left_wheel_target_rate.real;
      offset += sizeof(this->left_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_target_rate;
      u_right_wheel_target_rate.base = 0;
      u_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wheel_target_rate = u_right_wheel_target_rate.real;
      offset += sizeof(this->right_wheel_target_rate);
      union {
        float real;
        uint32_t base;
      } u_back_wheel_target_rate;
      u_back_wheel_target_rate.base = 0;
      u_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_back_wheel_target_rate.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->back_wheel_target_rate = u_back_wheel_target_rate.real;
      offset += sizeof(this->back_wheel_target_rate);
     return offset;
    }

    const char * getType(){ return "hardware_msg/HardwareCommand"; };
    const char * getMD5(){ return "373696df4a7275fc02befb0b8e656734"; };

  };

}
#endif