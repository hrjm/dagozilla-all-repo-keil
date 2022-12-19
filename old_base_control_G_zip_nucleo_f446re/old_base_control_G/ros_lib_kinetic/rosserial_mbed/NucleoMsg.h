#ifndef _ROS_dagozilla_msg_NucleoMsg_h
#define _ROS_dagozilla_msg_NucleoMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dagozilla_msg
{

  class NucleoMsg : public ros::Msg
  {
    public:
      typedef float _wheel_1_pos_type;
      _wheel_1_pos_type wheel_1_pos;
      typedef float _wheel_2_pos_type;
      _wheel_2_pos_type wheel_2_pos;
      typedef float _wheel_3_pos_type;
      _wheel_3_pos_type wheel_3_pos;
      typedef float _dribble_L_pos_type;
      _dribble_L_pos_type dribble_L_pos;
      typedef float _dribble_R_pos_type;
      _dribble_R_pos_type dribble_R_pos;
      typedef float _wheel_1_vel_type;
      _wheel_1_vel_type wheel_1_vel;
      typedef float _wheel_2_vel_type;
      _wheel_2_vel_type wheel_2_vel;
      typedef float _wheel_3_vel_type;
      _wheel_3_vel_type wheel_3_vel;
      typedef float _dribble_L_vel_type;
      _dribble_L_vel_type dribble_L_vel;
      typedef float _dribble_R_vel_type;
      _dribble_R_vel_type dribble_R_vel;
      typedef float _ball_distance_type;
      _ball_distance_type ball_distance;
      typedef int16_t _kick_mode_type;
      _kick_mode_type kick_mode;

    NucleoMsg():
      wheel_1_pos(0),
      wheel_2_pos(0),
      wheel_3_pos(0),
      dribble_L_pos(0),
      dribble_R_pos(0),
      wheel_1_vel(0),
      wheel_2_vel(0),
      wheel_3_vel(0),
      dribble_L_vel(0),
      dribble_R_vel(0),
      ball_distance(0),
      kick_mode(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_1_pos;
      u_wheel_1_pos.real = this->wheel_1_pos;
      *(outbuffer + offset + 0) = (u_wheel_1_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_1_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_1_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_1_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_1_pos);
      union {
        float real;
        uint32_t base;
      } u_wheel_2_pos;
      u_wheel_2_pos.real = this->wheel_2_pos;
      *(outbuffer + offset + 0) = (u_wheel_2_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_2_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_2_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_2_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_2_pos);
      union {
        float real;
        uint32_t base;
      } u_wheel_3_pos;
      u_wheel_3_pos.real = this->wheel_3_pos;
      *(outbuffer + offset + 0) = (u_wheel_3_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_3_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_3_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_3_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_3_pos);
      union {
        float real;
        uint32_t base;
      } u_dribble_L_pos;
      u_dribble_L_pos.real = this->dribble_L_pos;
      *(outbuffer + offset + 0) = (u_dribble_L_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribble_L_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribble_L_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribble_L_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribble_L_pos);
      union {
        float real;
        uint32_t base;
      } u_dribble_R_pos;
      u_dribble_R_pos.real = this->dribble_R_pos;
      *(outbuffer + offset + 0) = (u_dribble_R_pos.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribble_R_pos.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribble_R_pos.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribble_R_pos.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribble_R_pos);
      union {
        float real;
        uint32_t base;
      } u_wheel_1_vel;
      u_wheel_1_vel.real = this->wheel_1_vel;
      *(outbuffer + offset + 0) = (u_wheel_1_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_1_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_1_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_1_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_1_vel);
      union {
        float real;
        uint32_t base;
      } u_wheel_2_vel;
      u_wheel_2_vel.real = this->wheel_2_vel;
      *(outbuffer + offset + 0) = (u_wheel_2_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_2_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_2_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_2_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_2_vel);
      union {
        float real;
        uint32_t base;
      } u_wheel_3_vel;
      u_wheel_3_vel.real = this->wheel_3_vel;
      *(outbuffer + offset + 0) = (u_wheel_3_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_3_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_3_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_3_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_3_vel);
      union {
        float real;
        uint32_t base;
      } u_dribble_L_vel;
      u_dribble_L_vel.real = this->dribble_L_vel;
      *(outbuffer + offset + 0) = (u_dribble_L_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribble_L_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribble_L_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribble_L_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribble_L_vel);
      union {
        float real;
        uint32_t base;
      } u_dribble_R_vel;
      u_dribble_R_vel.real = this->dribble_R_vel;
      *(outbuffer + offset + 0) = (u_dribble_R_vel.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribble_R_vel.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribble_R_vel.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribble_R_vel.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribble_R_vel);
      union {
        float real;
        uint32_t base;
      } u_ball_distance;
      u_ball_distance.real = this->ball_distance;
      *(outbuffer + offset + 0) = (u_ball_distance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ball_distance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ball_distance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ball_distance.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ball_distance);
      union {
        int16_t real;
        uint16_t base;
      } u_kick_mode;
      u_kick_mode.real = this->kick_mode;
      *(outbuffer + offset + 0) = (u_kick_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kick_mode.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->kick_mode);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_1_pos;
      u_wheel_1_pos.base = 0;
      u_wheel_1_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_1_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_1_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_1_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_1_pos = u_wheel_1_pos.real;
      offset += sizeof(this->wheel_1_pos);
      union {
        float real;
        uint32_t base;
      } u_wheel_2_pos;
      u_wheel_2_pos.base = 0;
      u_wheel_2_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_2_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_2_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_2_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_2_pos = u_wheel_2_pos.real;
      offset += sizeof(this->wheel_2_pos);
      union {
        float real;
        uint32_t base;
      } u_wheel_3_pos;
      u_wheel_3_pos.base = 0;
      u_wheel_3_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_3_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_3_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_3_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_3_pos = u_wheel_3_pos.real;
      offset += sizeof(this->wheel_3_pos);
      union {
        float real;
        uint32_t base;
      } u_dribble_L_pos;
      u_dribble_L_pos.base = 0;
      u_dribble_L_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribble_L_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribble_L_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribble_L_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribble_L_pos = u_dribble_L_pos.real;
      offset += sizeof(this->dribble_L_pos);
      union {
        float real;
        uint32_t base;
      } u_dribble_R_pos;
      u_dribble_R_pos.base = 0;
      u_dribble_R_pos.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribble_R_pos.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribble_R_pos.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribble_R_pos.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribble_R_pos = u_dribble_R_pos.real;
      offset += sizeof(this->dribble_R_pos);
      union {
        float real;
        uint32_t base;
      } u_wheel_1_vel;
      u_wheel_1_vel.base = 0;
      u_wheel_1_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_1_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_1_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_1_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_1_vel = u_wheel_1_vel.real;
      offset += sizeof(this->wheel_1_vel);
      union {
        float real;
        uint32_t base;
      } u_wheel_2_vel;
      u_wheel_2_vel.base = 0;
      u_wheel_2_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_2_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_2_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_2_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_2_vel = u_wheel_2_vel.real;
      offset += sizeof(this->wheel_2_vel);
      union {
        float real;
        uint32_t base;
      } u_wheel_3_vel;
      u_wheel_3_vel.base = 0;
      u_wheel_3_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_3_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_3_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_3_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_3_vel = u_wheel_3_vel.real;
      offset += sizeof(this->wheel_3_vel);
      union {
        float real;
        uint32_t base;
      } u_dribble_L_vel;
      u_dribble_L_vel.base = 0;
      u_dribble_L_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribble_L_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribble_L_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribble_L_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribble_L_vel = u_dribble_L_vel.real;
      offset += sizeof(this->dribble_L_vel);
      union {
        float real;
        uint32_t base;
      } u_dribble_R_vel;
      u_dribble_R_vel.base = 0;
      u_dribble_R_vel.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dribble_R_vel.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dribble_R_vel.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dribble_R_vel.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dribble_R_vel = u_dribble_R_vel.real;
      offset += sizeof(this->dribble_R_vel);
      union {
        float real;
        uint32_t base;
      } u_ball_distance;
      u_ball_distance.base = 0;
      u_ball_distance.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ball_distance.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ball_distance.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ball_distance.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->ball_distance = u_ball_distance.real;
      offset += sizeof(this->ball_distance);
      union {
        int16_t real;
        uint16_t base;
      } u_kick_mode;
      u_kick_mode.base = 0;
      u_kick_mode.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kick_mode.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->kick_mode = u_kick_mode.real;
      offset += sizeof(this->kick_mode);
     return offset;
    }

    const char * getType(){ return "dagozilla_msg/NucleoMsg"; };
    const char * getMD5(){ return "4dda18750b6b7e9ad3ab0f53752128c8"; };

  };

}
#endif
