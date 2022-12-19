#ifndef _ROS_dagozilla_msg_NucleoCmd_h
#define _ROS_dagozilla_msg_NucleoCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msg
{

  class NucleoCmd : public ros::Msg
  {
    public:
      typedef float _wheel_1_vtarget_type;
      _wheel_1_vtarget_type wheel_1_vtarget;
      typedef float _wheel_2_vtarget_type;
      _wheel_2_vtarget_type wheel_2_vtarget;
      typedef float _wheel_3_vtarget_type;
      _wheel_3_vtarget_type wheel_3_vtarget;
      typedef float _dribble_L_vtarget_type;
      _dribble_L_vtarget_type dribble_L_vtarget;
      typedef float _dribble_R_vtarget_type;
      _dribble_R_vtarget_type dribble_R_vtarget;
      typedef float _kick_power_type;
      _kick_power_type kick_power;
      typedef int16_t _kick_mode_type;
      _kick_mode_type kick_mode;
      typedef int16_t _status_type;
      _status_type status;

    NucleoCmd():
      wheel_1_vtarget(0),
      wheel_2_vtarget(0),
      wheel_3_vtarget(0),
      dribble_L_vtarget(0),
      dribble_R_vtarget(0),
      kick_power(0),
      kick_mode(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_1_vtarget;
      u_wheel_1_vtarget.real = this->wheel_1_vtarget;
      *(outbuffer + offset + 0) = (u_wheel_1_vtarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_1_vtarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_1_vtarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_1_vtarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_1_vtarget);
      union {
        float real;
        uint32_t base;
      } u_wheel_2_vtarget;
      u_wheel_2_vtarget.real = this->wheel_2_vtarget;
      *(outbuffer + offset + 0) = (u_wheel_2_vtarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_2_vtarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_2_vtarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_2_vtarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_2_vtarget);
      union {
        float real;
        uint32_t base;
      } u_wheel_3_vtarget;
      u_wheel_3_vtarget.real = this->wheel_3_vtarget;
      *(outbuffer + offset + 0) = (u_wheel_3_vtarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_3_vtarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_3_vtarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_3_vtarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_3_vtarget);
      union {
        float real;
        uint32_t base;
      } u_dribble_L_vtarget;
      u_dribble_L_vtarget.real = this->dribble_L_vtarget;
      *(outbuffer + offset + 0) = (u_dribble_L_vtarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribble_L_vtarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribble_L_vtarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribble_L_vtarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribble_L_vtarget);
      union {
        float real;
        uint32_t base;
      } u_dribble_R_vtarget;
      u_dribble_R_vtarget.real = this->dribble_R_vtarget;
      *(outbuffer + offset + 0) = (u_dribble_R_vtarget.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribble_R_vtarget.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribble_R_vtarget.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribble_R_vtarget.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribble_R_vtarget);
      union {
        float real;
        uint32_t base;
      } u_kick_power;
      u_kick_power.real = this->kick_power;
      *(outbuffer + offset + 0) = (u_kick_power.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kick_power.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kick_power.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kick_power.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->kick_power);
      union {
        int16_t real;
        uint16_t base;
      } u_kick_mode;
      u_kick_mode.real = this->kick_mode;
      *(outbuffer + offset + 0) = (u_kick_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kick_mode.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->kick_mode);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_status.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_1_vtarget;
      u_wheel_1_vtarget.base = 0;
      u_wheel_1_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_1_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_1_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_1_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_1_vtarget = u_wheel_1_vtarget.real;
      offset += sizeof(this->wheel_1_vtarget);
      union {
        float real;
        uint32_t base;
      } u_wheel_2_vtarget;
      u_wheel_2_vtarget.base = 0;
      u_wheel_2_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_2_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_2_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_2_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_2_vtarget = u_wheel_2_vtarget.real;
      offset += sizeof(this->wheel_2_vtarget);
      union {
        float real;
        uint32_t base;
      } u_wheel_3_vtarget;
      u_wheel_3_vtarget.base = 0;
      u_wheel_3_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_3_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_3_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_3_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_3_vtarget = u_wheel_3_vtarget.real;
      offset += sizeof(this->wheel_3_vtarget);
      union {
        float real;
        uint32_t base;
      } u_dribble_L_vtarget;
      u_dribble_L_vtarget.base = 0;
      u_dribble_L_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribble_L_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribble_L_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribble_L_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribble_L_vtarget = u_dribble_L_vtarget.real;
      offset += sizeof(this->dribble_L_vtarget);
      union {
        float real;
        uint32_t base;
      } u_dribble_R_vtarget;
      u_dribble_R_vtarget.base = 0;
      u_dribble_R_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribble_R_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribble_R_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribble_R_vtarget.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribble_R_vtarget = u_dribble_R_vtarget.real;
      offset += sizeof(this->dribble_R_vtarget);
      union {
        float real;
        uint32_t base;
      } u_kick_power;
      u_kick_power.base = 0;
      u_kick_power.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kick_power.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kick_power.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kick_power.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->kick_power = u_kick_power.real;
      offset += sizeof(this->kick_power);
      union {
        int16_t real;
        uint16_t base;
      } u_kick_mode;
      u_kick_mode.base = 0;
      u_kick_mode.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kick_mode.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->kick_mode = u_kick_mode.real;
      offset += sizeof(this->kick_mode);
      union {
        int16_t real;
        uint16_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_status.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return "dagozilla_msg/NucleoCmd"; };
    const char * getMD5(){ return "83b22d5c1b2b9ca1b65eb136939d92a9"; };

  };

}
#endif
