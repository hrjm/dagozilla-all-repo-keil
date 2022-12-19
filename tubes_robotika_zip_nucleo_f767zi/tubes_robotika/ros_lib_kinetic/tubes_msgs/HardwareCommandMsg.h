#ifndef _ROS_tubes_msgs_HardwareCommandMsg_h
#define _ROS_tubes_msgs_HardwareCommandMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace tubes_msgs
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
      typedef bool _buzzer_state_type;
      _buzzer_state_type buzzer_state;

    HardwareCommandMsg():
      base_left_wheel_target_rate(0),
      base_right_wheel_target_rate(0),
      base_back_wheel_target_rate(0),
      buzzer_state(0)
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
        bool real;
        uint8_t base;
      } u_buzzer_state;
      u_buzzer_state.real = this->buzzer_state;
      *(outbuffer + offset + 0) = (u_buzzer_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->buzzer_state);
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
        bool real;
        uint8_t base;
      } u_buzzer_state;
      u_buzzer_state.base = 0;
      u_buzzer_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->buzzer_state = u_buzzer_state.real;
      offset += sizeof(this->buzzer_state);
     return offset;
    }

    const char * getType(){ return "tubes_msgs/HardwareCommandMsg"; };
    const char * getMD5(){ return "93a40d24404311e60e4bebe1ea76f5a2"; };

  };

}
#endif