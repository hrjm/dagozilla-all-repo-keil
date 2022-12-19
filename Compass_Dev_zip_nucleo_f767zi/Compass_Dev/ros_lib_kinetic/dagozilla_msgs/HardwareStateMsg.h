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
      typedef float _dribbler_left_wheel_velocity_type;
      _dribbler_left_wheel_velocity_type dribbler_left_wheel_velocity;
      typedef float _dribbler_right_wheel_velocity_type;
      _dribbler_right_wheel_velocity_type dribbler_right_wheel_velocity;
      typedef float _dribbler_left_potentio_value_type;
      _dribbler_left_potentio_value_type dribbler_left_potentio_value;
      typedef float _dribbler_right_potentio_value_type;
      _dribbler_right_potentio_value_type dribbler_right_potentio_value;
      typedef float _dribbler_ir_distance_type;
      _dribbler_ir_distance_type dribbler_ir_distance;
      typedef float _compass_angle_type;
      _compass_angle_type compass_angle;

    HardwareStateMsg():
      base_left_wheel_position(0),
      base_right_wheel_position(0),
      base_back_wheel_position(0),
      base_left_wheel_velocity(0),
      base_right_wheel_velocity(0),
      base_back_wheel_velocity(0),
      dribbler_left_wheel_velocity(0),
      dribbler_right_wheel_velocity(0),
      dribbler_left_potentio_value(0),
      dribbler_right_potentio_value(0),
      dribbler_ir_distance(0),
      compass_angle(0)
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
      union {
        float real;
        uint32_t base;
      } u_dribbler_left_wheel_velocity;
      u_dribbler_left_wheel_velocity.real = this->dribbler_left_wheel_velocity;
      *(outbuffer + offset + 0) = (u_dribbler_left_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_left_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_left_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_left_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_left_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_dribbler_right_wheel_velocity;
      u_dribbler_right_wheel_velocity.real = this->dribbler_right_wheel_velocity;
      *(outbuffer + offset + 0) = (u_dribbler_right_wheel_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_right_wheel_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_right_wheel_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_right_wheel_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_right_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_dribbler_left_potentio_value;
      u_dribbler_left_potentio_value.real = this->dribbler_left_potentio_value;
      *(outbuffer + offset + 0) = (u_dribbler_left_potentio_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_left_potentio_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_left_potentio_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_left_potentio_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_left_potentio_value);
      union {
        float real;
        uint32_t base;
      } u_dribbler_right_potentio_value;
      u_dribbler_right_potentio_value.real = this->dribbler_right_potentio_value;
      *(outbuffer + offset + 0) = (u_dribbler_right_potentio_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_right_potentio_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_right_potentio_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_right_potentio_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_right_potentio_value);
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
      union {
        float real;
        uint32_t base;
      } u_dribbler_left_wheel_velocity;
      u_dribbler_left_wheel_velocity.base = 0;
      u_dribbler_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_left_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_left_wheel_velocity = u_dribbler_left_wheel_velocity.real;
      offset += sizeof(this->dribbler_left_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_dribbler_right_wheel_velocity;
      u_dribbler_right_wheel_velocity.base = 0;
      u_dribbler_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_right_wheel_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_right_wheel_velocity = u_dribbler_right_wheel_velocity.real;
      offset += sizeof(this->dribbler_right_wheel_velocity);
      union {
        float real;
        uint32_t base;
      } u_dribbler_left_potentio_value;
      u_dribbler_left_potentio_value.base = 0;
      u_dribbler_left_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_left_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_left_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_left_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_left_potentio_value = u_dribbler_left_potentio_value.real;
      offset += sizeof(this->dribbler_left_potentio_value);
      union {
        float real;
        uint32_t base;
      } u_dribbler_right_potentio_value;
      u_dribbler_right_potentio_value.base = 0;
      u_dribbler_right_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribbler_right_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribbler_right_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribbler_right_potentio_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribbler_right_potentio_value = u_dribbler_right_potentio_value.real;
      offset += sizeof(this->dribbler_right_potentio_value);
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
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/HardwareStateMsg"; };
    const char * getMD5(){ return "5ba707b0fc24baadcd4686a4caa3d712"; };

  };

}
#endif
