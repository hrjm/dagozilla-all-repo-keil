#ifndef _ROS_dagozilla_msgs_StaticBodyMsg_h
#define _ROS_dagozilla_msgs_StaticBodyMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dagozilla_msgs/VectorMsg.h"
#include "dagozilla_msgs/AngleMsg.h"

namespace dagozilla_msgs
{

  class StaticBodyMsg : public ros::Msg
  {
    public:
      typedef dagozilla_msgs::VectorMsg _position_type;
      _position_type position;
      typedef dagozilla_msgs::AngleMsg _orientation_type;
      _orientation_type orientation;

    StaticBodyMsg():
      position(),
      orientation()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "dagozilla_msgs/StaticBodyMsg"; };
    const char * getMD5(){ return "73e748129d10802ca00ec9d150b0d999"; };

  };

}
#endif