#ifndef KIBA_CONTROL_KIBA_HW_SIM_H
#define KIBA_CONTROL_KIBA_HW_SIM_H

#include <ros_control_boilerplate/sim_hw_interface.h>

namespace kiba_control
{
  class KibaHwSim : public ros_control_boilerplate::SimHWInterface
  {
    public:
      KibaHwSim(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL) :
        ros_control_boilerplate::SimHWInterface(nh, urdf_model)
      {
        // FIXME: read default joint values
  
      }

      void init()
      {
        ros_control_boilerplate::SimHWInterface::init();
      }
  };
}
#endif
