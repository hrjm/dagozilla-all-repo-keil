#ifndef _ROS_tubes_msgs_HardwareStateMsg_h
#define _ROS_tubes_msgs_HardwareStateMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tubes_msgs
{

  class HardwareStateMsg : public ros::Msg
  {
    public:
      typedef float _base_left_wheel_position_type;
      _base_left_wheel_position_type base_left_wheel_position;
      typedef float _base_right_wheel_position_type;
      _base_right_wheel_position_type base_right_wheel_position;
      typedef float _base_back_wheel_position_type;
      _base_back_wheel_position_type base_back_wheel_position;
      typedef float _base_left_wheel_velocity_type;
      _base_left_wheel_velocity_type base_left_wheel_velocity;
      typedef float _base_right_wheel_velocity_type;
      _base_right_wheel_velocity_type base_right_wheel_velocity;
      typedef float _base_back_wheel_velocity_type;
      _base_back_wheel_velocity_type base_back_wheel_velocity;

    HardwareStateMsg():
      base_left_wheel_position(0),
      base_right_wheel_position(0),
      base_back_wheel_position(0),
      base_left_wheel_velocity(0),
      base_right_wheel_velocity(0),
      base_back_wheel_velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_left_wheel_position;
      u_base_left_wheel_position.real = this->base_left_wheel_position;
      *(outbuffer + offset + 0) = (u_base_left_wheel_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_left_wheel_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_left_wheel_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_left_wheel_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_left_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_base_right_wheel_position;
      u_base_right_wheel_position.real = this->base_right_wheel_position;
      *(outbuffer + offset + 0) = (u_base_right_wheel_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_right_wheel_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_right_wheel_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_right_wheel_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_right_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_base_back_wheel_position;
      u_base_back_wheel_position.real = this->base_back_wheel_position;
      *(outbuffer + offset + 0) = (u_base_back_wheel_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_back_wheel_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_back_wheel_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_back_wheel_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_back_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_base_left_wheel_velocity;
      u_base_left_wheel_velocity.real = this->base_left_wheel_velocity;
      *(outbuffer + offset + 0) = (u_base_left_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_left_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_left_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_left_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_left_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_right_wheel_velocity;
      u_base_right_wheel_velocity.real = this->base_right_wheel_velocity;
      *(outbuffer + offset + 0) = (u_base_right_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_right_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_right_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_right_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_right_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_back_wheel_velocity;
      u_base_back_wheel_velocity.real = this->base_back_wheel_velocity;
      *(outbuffer + offset + 0) = (u_base_back_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_back_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_back_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_back_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_back_wheel_velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_left_wheel_position;
      u_base_left_wheel_position.base = 0;
      u_base_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_left_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_left_wheel_position = u_base_left_wheel_position.real;
      offset += sizeof(this->base_left_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_base_right_wheel_position;
      u_base_right_wheel_position.base = 0;
      u_base_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_right_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_right_wheel_position = u_base_right_wheel_position.real;
      offset += sizeof(this->base_right_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_base_back_wheel_position;
      u_base_back_wheel_position.base = 0;
      u_base_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_back_wheel_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_back_wheel_position = u_base_back_wheel_position.real;
      offset += sizeof(this->base_back_wheel_position);
      union {
        float real;
        uint32_t base;
      } u_base_left_wheel_velocity;
      u_base_left_wheel_velocity.base = 0;
      u_base_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_left_wheel_velocity = u_base_left_wheel_velocity.real;
      offset += sizeof(this->base_left_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_right_wheel_velocity;
      u_base_right_wheel_velocity.base = 0;
      u_base_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_right_wheel_velocity = u_base_right_wheel_velocity.real;
      offset += sizeof(this->base_right_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_base_back_wheel_velocity;
      u_base_back_wheel_velocity.base = 0;
      u_base_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_back_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_back_wheel_velocity = u_base_back_wheel_velocity.real;
      offset += sizeof(this->base_back_wheel_velocity);
     return offset;
    }

    const char * getType(){ return "tubes_msgs/HardwareStateMsg"; };
    const char * getMD5(){ return "f6cb32689bc35558f2aa933c41184a77"; };

  };

}
#endif