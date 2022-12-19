#ifndef _ROS_dgz_msgs_HardwareCommand_h
#define _ROS_dgz_msgs_HardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dgz_msgs
{

  class HardwareCommand : public ros::Msg
  {
    public:
      uint32_t base_motors_pwm_length;
      typedef double _base_motors_pwm_type;
      _base_motors_pwm_type st_base_motors_pwm;
      _base_motors_pwm_type * base_motors_pwm;
      uint32_t dribbler_motors_pwm_length;
      typedef double _dribbler_motors_pwm_type;
      _dribbler_motors_pwm_type st_dribbler_motors_pwm;
      _dribbler_motors_pwm_type * dribbler_motors_pwm;
      typedef double _kicker_pwm_type;
      _kicker_pwm_type kicker_pwm;
      typedef bool _is_shoot_type;
      _is_shoot_type is_shoot;

    HardwareCommand():
      base_motors_pwm_length(0), st_base_motors_pwm(), base_motors_pwm(nullptr),
      dribbler_motors_pwm_length(0), st_dribbler_motors_pwm(), dribbler_motors_pwm(nullptr),
      kicker_pwm(0),
      is_shoot(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->base_motors_pwm_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base_motors_pwm_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->base_motors_pwm_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->base_motors_pwm_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motors_pwm_length);
      for( uint32_t i = 0; i < base_motors_pwm_length; i++){
      union {
        double real;
        uint64_t base;
      } u_base_motors_pwmi;
      u_base_motors_pwmi.real = this->base_motors_pwm[i];
      *(outbuffer + offset + 0) = (u_base_motors_pwmi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motors_pwmi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motors_pwmi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motors_pwmi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_base_motors_pwmi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_base_motors_pwmi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_base_motors_pwmi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_base_motors_pwmi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->base_motors_pwm[i]);
      }
      *(outbuffer + offset + 0) = (this->dribbler_motors_pwm_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dribbler_motors_pwm_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dribbler_motors_pwm_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dribbler_motors_pwm_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motors_pwm_length);
      for( uint32_t i = 0; i < dribbler_motors_pwm_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dribbler_motors_pwmi;
      u_dribbler_motors_pwmi.real = this->dribbler_motors_pwm[i];
      *(outbuffer + offset + 0) = (u_dribbler_motors_pwmi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motors_pwmi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motors_pwmi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motors_pwmi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dribbler_motors_pwmi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dribbler_motors_pwmi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dribbler_motors_pwmi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dribbler_motors_pwmi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dribbler_motors_pwm[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_kicker_pwm;
      u_kicker_pwm.real = this->kicker_pwm;
      *(outbuffer + offset + 0) = (u_kicker_pwm.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_kicker_pwm.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_kicker_pwm.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_kicker_pwm.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_kicker_pwm.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_kicker_pwm.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_kicker_pwm.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_kicker_pwm.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->kicker_pwm);
      union {
        bool real;
        uint8_t base;
      } u_is_shoot;
      u_is_shoot.real = this->is_shoot;
      *(outbuffer + offset + 0) = (u_is_shoot.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_shoot);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t base_motors_pwm_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      base_motors_pwm_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      base_motors_pwm_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      base_motors_pwm_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->base_motors_pwm_length);
      if(base_motors_pwm_lengthT > base_motors_pwm_length)
        this->base_motors_pwm = (double*)realloc(this->base_motors_pwm, base_motors_pwm_lengthT * sizeof(double));
      base_motors_pwm_length = base_motors_pwm_lengthT;
      for( uint32_t i = 0; i < base_motors_pwm_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_base_motors_pwm;
      u_st_base_motors_pwm.base = 0;
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_base_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_base_motors_pwm = u_st_base_motors_pwm.real;
      offset += sizeof(this->st_base_motors_pwm);
        memcpy( &(this->base_motors_pwm[i]), &(this->st_base_motors_pwm), sizeof(double));
      }
      uint32_t dribbler_motors_pwm_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dribbler_motors_pwm_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dribbler_motors_pwm_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dribbler_motors_pwm_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dribbler_motors_pwm_length);
      if(dribbler_motors_pwm_lengthT > dribbler_motors_pwm_length)
        this->dribbler_motors_pwm = (double*)realloc(this->dribbler_motors_pwm, dribbler_motors_pwm_lengthT * sizeof(double));
      dribbler_motors_pwm_length = dribbler_motors_pwm_lengthT;
      for( uint32_t i = 0; i < dribbler_motors_pwm_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dribbler_motors_pwm;
      u_st_dribbler_motors_pwm.base = 0;
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dribbler_motors_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dribbler_motors_pwm = u_st_dribbler_motors_pwm.real;
      offset += sizeof(this->st_dribbler_motors_pwm);
        memcpy( &(this->dribbler_motors_pwm[i]), &(this->st_dribbler_motors_pwm), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_kicker_pwm;
      u_kicker_pwm.base = 0;
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_kicker_pwm.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->kicker_pwm = u_kicker_pwm.real;
      offset += sizeof(this->kicker_pwm);
      union {
        bool real;
        uint8_t base;
      } u_is_shoot;
      u_is_shoot.base = 0;
      u_is_shoot.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_shoot = u_is_shoot.real;
      offset += sizeof(this->is_shoot);
     return offset;
    }

    virtual const char * getType() override { return "dgz_msgs/HardwareCommand"; };
    virtual const char * getMD5() override { return "6ffed4cb64509a95dde8ebbc7ac3aa00"; };

  };

}
#endif
