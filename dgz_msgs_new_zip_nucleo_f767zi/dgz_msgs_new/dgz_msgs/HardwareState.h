#ifndef _ROS_dgz_msgs_HardwareState_h
#define _ROS_dgz_msgs_HardwareState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dgz_msgs
{

  class HardwareState : public ros::Msg
  {
    public:
      uint32_t base_motors_pulse_delta_length;
      typedef int32_t _base_motors_pulse_delta_type;
      _base_motors_pulse_delta_type st_base_motors_pulse_delta;
      _base_motors_pulse_delta_type * base_motors_pulse_delta;
      uint32_t base_encoders_pulse_delta_length;
      typedef int32_t _base_encoders_pulse_delta_type;
      _base_encoders_pulse_delta_type st_base_encoders_pulse_delta;
      _base_encoders_pulse_delta_type * base_encoders_pulse_delta;
      uint32_t dribbler_motors_pulse_delta_length;
      typedef int32_t _dribbler_motors_pulse_delta_type;
      _dribbler_motors_pulse_delta_type st_dribbler_motors_pulse_delta;
      _dribbler_motors_pulse_delta_type * dribbler_motors_pulse_delta;
      uint32_t dribbler_potentios_reading_length;
      typedef double _dribbler_potentios_reading_type;
      _dribbler_potentios_reading_type st_dribbler_potentios_reading;
      _dribbler_potentios_reading_type * dribbler_potentios_reading;
      typedef double _compass_reading_type;
      _compass_reading_type compass_reading;
      typedef double _ir_reading_type;
      _ir_reading_type ir_reading;

    HardwareState():
      base_motors_pulse_delta_length(0), st_base_motors_pulse_delta(), base_motors_pulse_delta(nullptr),
      base_encoders_pulse_delta_length(0), st_base_encoders_pulse_delta(), base_encoders_pulse_delta(nullptr),
      dribbler_motors_pulse_delta_length(0), st_dribbler_motors_pulse_delta(), dribbler_motors_pulse_delta(nullptr),
      dribbler_potentios_reading_length(0), st_dribbler_potentios_reading(), dribbler_potentios_reading(nullptr),
      compass_reading(0),
      ir_reading(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->base_motors_pulse_delta_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base_motors_pulse_delta_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->base_motors_pulse_delta_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->base_motors_pulse_delta_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motors_pulse_delta_length);
      for( uint32_t i = 0; i < base_motors_pulse_delta_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_base_motors_pulse_deltai;
      u_base_motors_pulse_deltai.real = this->base_motors_pulse_delta[i];
      *(outbuffer + offset + 0) = (u_base_motors_pulse_deltai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_motors_pulse_deltai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_motors_pulse_deltai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_motors_pulse_deltai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_motors_pulse_delta[i]);
      }
      *(outbuffer + offset + 0) = (this->base_encoders_pulse_delta_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->base_encoders_pulse_delta_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->base_encoders_pulse_delta_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->base_encoders_pulse_delta_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoders_pulse_delta_length);
      for( uint32_t i = 0; i < base_encoders_pulse_delta_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_base_encoders_pulse_deltai;
      u_base_encoders_pulse_deltai.real = this->base_encoders_pulse_delta[i];
      *(outbuffer + offset + 0) = (u_base_encoders_pulse_deltai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_base_encoders_pulse_deltai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_base_encoders_pulse_deltai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_base_encoders_pulse_deltai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->base_encoders_pulse_delta[i]);
      }
      *(outbuffer + offset + 0) = (this->dribbler_motors_pulse_delta_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dribbler_motors_pulse_delta_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dribbler_motors_pulse_delta_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dribbler_motors_pulse_delta_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motors_pulse_delta_length);
      for( uint32_t i = 0; i < dribbler_motors_pulse_delta_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_dribbler_motors_pulse_deltai;
      u_dribbler_motors_pulse_deltai.real = this->dribbler_motors_pulse_delta[i];
      *(outbuffer + offset + 0) = (u_dribbler_motors_pulse_deltai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_motors_pulse_deltai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_motors_pulse_deltai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_motors_pulse_deltai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_motors_pulse_delta[i]);
      }
      *(outbuffer + offset + 0) = (this->dribbler_potentios_reading_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dribbler_potentios_reading_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dribbler_potentios_reading_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dribbler_potentios_reading_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dribbler_potentios_reading_length);
      for( uint32_t i = 0; i < dribbler_potentios_reading_length; i++){
      union {
        double real;
        uint64_t base;
      } u_dribbler_potentios_readingi;
      u_dribbler_potentios_readingi.real = this->dribbler_potentios_reading[i];
      *(outbuffer + offset + 0) = (u_dribbler_potentios_readingi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dribbler_potentios_readingi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dribbler_potentios_readingi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dribbler_potentios_readingi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_dribbler_potentios_readingi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_dribbler_potentios_readingi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_dribbler_potentios_readingi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_dribbler_potentios_readingi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->dribbler_potentios_reading[i]);
      }
      union {
        double real;
        uint64_t base;
      } u_compass_reading;
      u_compass_reading.real = this->compass_reading;
      *(outbuffer + offset + 0) = (u_compass_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_compass_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_compass_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_compass_reading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_compass_reading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_compass_reading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_compass_reading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_compass_reading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->compass_reading);
      union {
        double real;
        uint64_t base;
      } u_ir_reading;
      u_ir_reading.real = this->ir_reading;
      *(outbuffer + offset + 0) = (u_ir_reading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ir_reading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ir_reading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ir_reading.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ir_reading.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ir_reading.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ir_reading.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ir_reading.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ir_reading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t base_motors_pulse_delta_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      base_motors_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      base_motors_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      base_motors_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->base_motors_pulse_delta_length);
      if(base_motors_pulse_delta_lengthT > base_motors_pulse_delta_length)
        this->base_motors_pulse_delta = (int32_t*)realloc(this->base_motors_pulse_delta, base_motors_pulse_delta_lengthT * sizeof(int32_t));
      base_motors_pulse_delta_length = base_motors_pulse_delta_lengthT;
      for( uint32_t i = 0; i < base_motors_pulse_delta_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_base_motors_pulse_delta;
      u_st_base_motors_pulse_delta.base = 0;
      u_st_base_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_base_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_base_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_base_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_base_motors_pulse_delta = u_st_base_motors_pulse_delta.real;
      offset += sizeof(this->st_base_motors_pulse_delta);
        memcpy( &(this->base_motors_pulse_delta[i]), &(this->st_base_motors_pulse_delta), sizeof(int32_t));
      }
      uint32_t base_encoders_pulse_delta_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      base_encoders_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      base_encoders_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      base_encoders_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->base_encoders_pulse_delta_length);
      if(base_encoders_pulse_delta_lengthT > base_encoders_pulse_delta_length)
        this->base_encoders_pulse_delta = (int32_t*)realloc(this->base_encoders_pulse_delta, base_encoders_pulse_delta_lengthT * sizeof(int32_t));
      base_encoders_pulse_delta_length = base_encoders_pulse_delta_lengthT;
      for( uint32_t i = 0; i < base_encoders_pulse_delta_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_base_encoders_pulse_delta;
      u_st_base_encoders_pulse_delta.base = 0;
      u_st_base_encoders_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_base_encoders_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_base_encoders_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_base_encoders_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_base_encoders_pulse_delta = u_st_base_encoders_pulse_delta.real;
      offset += sizeof(this->st_base_encoders_pulse_delta);
        memcpy( &(this->base_encoders_pulse_delta[i]), &(this->st_base_encoders_pulse_delta), sizeof(int32_t));
      }
      uint32_t dribbler_motors_pulse_delta_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dribbler_motors_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dribbler_motors_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dribbler_motors_pulse_delta_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dribbler_motors_pulse_delta_length);
      if(dribbler_motors_pulse_delta_lengthT > dribbler_motors_pulse_delta_length)
        this->dribbler_motors_pulse_delta = (int32_t*)realloc(this->dribbler_motors_pulse_delta, dribbler_motors_pulse_delta_lengthT * sizeof(int32_t));
      dribbler_motors_pulse_delta_length = dribbler_motors_pulse_delta_lengthT;
      for( uint32_t i = 0; i < dribbler_motors_pulse_delta_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_dribbler_motors_pulse_delta;
      u_st_dribbler_motors_pulse_delta.base = 0;
      u_st_dribbler_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dribbler_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dribbler_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dribbler_motors_pulse_delta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_dribbler_motors_pulse_delta = u_st_dribbler_motors_pulse_delta.real;
      offset += sizeof(this->st_dribbler_motors_pulse_delta);
        memcpy( &(this->dribbler_motors_pulse_delta[i]), &(this->st_dribbler_motors_pulse_delta), sizeof(int32_t));
      }
      uint32_t dribbler_potentios_reading_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dribbler_potentios_reading_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dribbler_potentios_reading_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dribbler_potentios_reading_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dribbler_potentios_reading_length);
      if(dribbler_potentios_reading_lengthT > dribbler_potentios_reading_length)
        this->dribbler_potentios_reading = (double*)realloc(this->dribbler_potentios_reading, dribbler_potentios_reading_lengthT * sizeof(double));
      dribbler_potentios_reading_length = dribbler_potentios_reading_lengthT;
      for( uint32_t i = 0; i < dribbler_potentios_reading_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_dribbler_potentios_reading;
      u_st_dribbler_potentios_reading.base = 0;
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_dribbler_potentios_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_dribbler_potentios_reading = u_st_dribbler_potentios_reading.real;
      offset += sizeof(this->st_dribbler_potentios_reading);
        memcpy( &(this->dribbler_potentios_reading[i]), &(this->st_dribbler_potentios_reading), sizeof(double));
      }
      union {
        double real;
        uint64_t base;
      } u_compass_reading;
      u_compass_reading.base = 0;
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_compass_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->compass_reading = u_compass_reading.real;
      offset += sizeof(this->compass_reading);
      union {
        double real;
        uint64_t base;
      } u_ir_reading;
      u_ir_reading.base = 0;
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ir_reading.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ir_reading = u_ir_reading.real;
      offset += sizeof(this->ir_reading);
     return offset;
    }

    virtual const char * getType() override { return "dgz_msgs/HardwareState"; };
    virtual const char * getMD5() override { return "0e1d60ebe47a3a5c461d935d0bd91b96"; };

  };

}
#endif
