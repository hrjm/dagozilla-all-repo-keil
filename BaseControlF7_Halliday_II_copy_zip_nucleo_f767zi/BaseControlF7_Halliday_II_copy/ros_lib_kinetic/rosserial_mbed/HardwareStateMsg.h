#ifndef _ROS_dagozilla_msgs_HardwareStateMsg_h
#define _ROS_dagozilla_msgs_HardwareStateMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msgs
{

  class HardwareStateMsg : public ros::Msg
  {
    public:
      typedef float _base_wheel_left_position_type;
      _base_wheel_left_position_type base_wheel_left_position;
      typedef float _base_wheel_right_position_type;
      _base_wheel_right_position_type base_wheel_right_position;
      typedef float _base_wheel_back_position_type;
      _base_wheel_back_position_type base_wheel_back_position;
      typedef float _base_wheel_left_velocity_type;
      _base_wheel_left_velocity_type base_wheel_left_velocity;
      typedef float _base_wheel_right_velocity_type;
      _base_wheel_right_velocity_type base_wheel_right_velocity;
      typedef float _base_wheel_back_velocity_type;
      _base_wheel_back_velocity_type base_wheel_back_velocity;
      typedef float _dribbler_ir_distance_type;
      _dribbler_ir_distance_type dribbler_ir_distance;
      typedef float _compass_angle_type;
      _compass_angle_type compass_angle;
      typedef bool _dribbler_active_type;
      _dribbler_active_type dribbler_active;

    HardwareStateMsg():
      base_wheel_left_position(0),
      base_wheel_right_position(0),
      base_wheel_back_position(0),
      base_wheel_left_velocity(0),
      base_wheel_right_velocity(0),
      base_wheel_back_velocity(0),
      dribbler_ir_distance(0),
      compass_angle(0),
      dribbler_active(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_base_wheel_left_position;
      u_base_wheel_left_position.real = this->base_wheel_left_position;
      *(outbuffer + offset + 0) = (u_base_wheel_left_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_wheel_left_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_wheel_left_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_wheel_left_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_wheel_left_position);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_right_position;
      u_base_wheel_right_position.real = this->base_wheel_right_position;
      *(outbuffer + offset + 0) = (u_base_wheel_right_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_wheel_right_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_wheel_right_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_wheel_right_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_wheel_right_position);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_back_position;
      u_base_wheel_back_position.real = this->base_wheel_back_position;
      *(outbuffer + offset + 0) = (u_base_wheel_back_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_wheel_back_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_wheel_back_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_wheel_back_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_wheel_back_position);
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
      } u_dribbler_ir_distance;
      u_dribbler_ir_distance.real = this->dribbler_ir_distance;
      *(outbuffer + offset + 0) = (u_dribbler_ir_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_ir_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_ir_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_ir_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_ir_distance);
      union {
        float real;
        uint32_t base;
      } u_compass_angle;
      u_compass_angle.real = this->compass_angle;
      *(outbuffer + offset + 0) = (u_compass_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_compass_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_compass_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_compass_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->compass_angle);
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
      } u_base_wheel_left_position;
      u_base_wheel_left_position.base = 0;
      u_base_wheel_left_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_wheel_left_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_wheel_left_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_wheel_left_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_wheel_left_position = u_base_wheel_left_position.real;
      offset += sizeof(this->base_wheel_left_position);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_right_position;
      u_base_wheel_right_position.base = 0;
      u_base_wheel_right_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_wheel_right_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_wheel_right_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_wheel_right_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_wheel_right_position = u_base_wheel_right_position.real;
      offset += sizeof(this->base_wheel_right_position);
      union {
        float real;
        uint32_t base;
      } u_base_wheel_back_position;
      u_base_wheel_back_position.base = 0;
      u_base_wheel_back_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_base_wheel_back_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_base_wheel_back_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_base_wheel_back_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->base_wheel_back_position = u_base_wheel_back_position.real;
      offset += sizeof(this->base_wheel_back_position);
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
      } u_dribbler_ir_distance;
      u_dribbler_ir_distance.base = 0;
      u_dribbler_ir_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_ir_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_ir_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_ir_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_ir_distance = u_dribbler_ir_distance.real;
      offset += sizeof(this->dribbler_ir_distance);
      union {
        float real;
        uint32_t base;
      } u_compass_angle;
      u_compass_angle.base = 0;
      u_compass_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_compass_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_compass_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_compass_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->compass_angle = u_compass_angle.real;
      offset += sizeof(this->compass_angle);
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

    const char * getType(){ return "dagozilla_msgs/HardwareStateMsg"; };
    const char * getMD5(){ return "55139ad38578ece61474c70e0b7976a3"; };

  };

}
#endif