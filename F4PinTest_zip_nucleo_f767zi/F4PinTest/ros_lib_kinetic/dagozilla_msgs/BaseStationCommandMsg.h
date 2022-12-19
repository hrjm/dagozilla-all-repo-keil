#ifndef _ROS_dagozilla_msgs_BaseStationCommandMsg_h
#define _ROS_dagozilla_msgs_BaseStationCommandMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dagozilla_msgs/StaticBodyMsg.h"

namespace dagozilla_msgs
{

  class BaseStationCommandMsg : public ros::Msg
  {
    public:
      typedef uint16_t _state_type;
      _state_type state;
      typedef uint8_t _modifier_type;
      _modifier_type modifier;
      typedef dagozilla_msgs::StaticBodyMsg _pose_type;
      _pose_type pose;

    BaseStationCommandMsg():
      state(0),
      modifier(0),
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->state >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->state >> (8 * 1)) & 0xFF;
      offset += sizeof(this->state);
      *(outbuffer + offset + 0) = (this->modifier >> (8 * 0)) & 0xFF;
      offset += sizeof(this->modifier);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->state =  ((uint16_t) (*(inbuffer + offset)));
      this->state |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->state);
      this->modifier =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->modifier);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/BaseStationCommandMsg"; };
    const char * getMD5(){ return "a219f0d53d414890337ae0486a46a9b7"; };

  };

}
#endif