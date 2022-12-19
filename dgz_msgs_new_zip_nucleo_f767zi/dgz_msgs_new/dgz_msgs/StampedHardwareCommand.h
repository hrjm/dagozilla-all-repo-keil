#ifndef _ROS_dgz_msgs_StampedHardwareCommand_h
#define _ROS_dgz_msgs_StampedHardwareCommand_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "dgz_msgs/HardwareCommand.h"

namespace dgz_msgs
{

  class StampedHardwareCommand : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef dgz_msgs::HardwareCommand _data_type;
      _data_type data;

    StampedHardwareCommand():
      header(),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->data.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->data.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "dgz_msgs/StampedHardwareCommand"; };
    virtual const char * getMD5() override { return "69dc82ba4bdd2a329158229b683ac6af"; };

  };

}
#endif
