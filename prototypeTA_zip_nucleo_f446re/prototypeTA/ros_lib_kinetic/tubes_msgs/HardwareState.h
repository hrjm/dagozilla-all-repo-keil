#ifndef _ROS_hardware_msg_HardwareState_h
#define _ROS_hardware_msg_HardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace hardware_msg
{

  class HardwareState : public ros::Msg
  {
    public:
      typedef float _left_wheel_position_type;
      _left_wheel_position_type left_wheel_position;
      typedef float _right_wheel_position_type;
      _right_wheel_position_type right_wheel_position;
      typedef float _back_wheel_position_type;
      _back_wheel_position_type back_wheel_position;
      typedef float _left_wheel_velocity_type;
      _left_wheel_velocity_type left_wheel_velocity;
      typedef float _right_wheel_velocity_type;
      _right_wheel_velocity_type right_wheel_velocity;
      typedef float _back_wheel_velocity_type;
      _back_wheel_velocity_type back_wheel_velocity;

    HardwareState():
      left_wheel_position(0),
      right_wheel_position(0),
      back_wheel_position(0),
      left_wheel_velocity(0),
      right_wheel_velocity(0),
      back_wheel_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_position;
      u_left_wheel_position.real = this->left_wheel_position;
      *(outbuffer + offset + 0) = (u_left_wheel_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_position;
      u_right_wheel_position.real = this->right_wheel_position;
      *(outbuffer + offset + 0) = (u_right_wheel_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_back_wheel_position;
      u_back_wheel_position.real = this->back_wheel_position;
      *(outbuffer + offset + 0) = (u_back_wheel_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_back_wheel_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_back_wheel_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_back_wheel_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->back_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_left_wheel_velocity;
      u_left_wheel_velocity.real = this->left_wheel_velocity;
      *(outbuffer + offset + 0) = (u_left_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_velocity;
      u_right_wheel_velocity.real = this->right_wheel_velocity;
      *(outbuffer + offset + 0) = (u_right_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_back_wheel_velocity;
      u_back_wheel_velocity.real = this->back_wheel_velocity;
      *(outbuffer + offset + 0) = (u_back_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_back_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_back_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_back_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->back_wheel_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left_wheel_position;
      u_left_wheel_position.base = 0;
      u_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_wheel_position = u_left_wheel_position.real;
      offset += sizeof(this->left_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_position;
      u_right_wheel_position.base = 0;
      u_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wheel_position = u_right_wheel_position.real;
      offset += sizeof(this->right_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_back_wheel_position;
      u_back_wheel_position.base = 0;
      u_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->back_wheel_position = u_back_wheel_position.real;
      offset += sizeof(this->back_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_left_wheel_velocity;
      u_left_wheel_velocity.base = 0;
      u_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left_wheel_velocity = u_left_wheel_velocity.real;
      offset += sizeof(this->left_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_right_wheel_velocity;
      u_right_wheel_velocity.base = 0;
      u_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right_wheel_velocity = u_right_wheel_velocity.real;
      offset += sizeof(this->right_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_back_wheel_velocity;
      u_back_wheel_velocity.base = 0;
      u_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->back_wheel_velocity = u_back_wheel_velocity.real;
      offset += sizeof(this->back_wheel_velocity);
     return offset;
    }

    const char * getType(){ return "hardware_msg/HardwareState"; };
    const char * getMD5(){ return "1a2eb6e6c45134bc21d28d45f463a242"; };

  };

}
#endif