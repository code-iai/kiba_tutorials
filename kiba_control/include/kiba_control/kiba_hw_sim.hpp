#ifndef KIBA_CONTROL_KIBA_HW_SIM_H
#define KIBA_CONTROL_KIBA_HW_SIM_H

#include <ros_control_boilerplate/sim_hw_interface.h>
#include <kiba_control/utils.hpp>

namespace kiba_control
{
  class KibaHwSim : public ros_control_boilerplate::SimHWInterface
  {
    public:
      KibaHwSim(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL) :
        ros_control_boilerplate::SimHWInterface(nh, urdf_model), nh_(nh)
      {
      }

      void init()
      {
        ros_control_boilerplate::SimHWInterface::init();

        // read and set init configuration
        std::map<std::string, double> init_config =
          readParam< std::map<std::string, double> >(nh_, "init_config");

        for (size_t i=0; i<joint_names_.size(); ++i)
        {
          joint_position_[i] = init_config.find(joint_names_[i])->second;
          joint_position_command_[i] = joint_position_[i];
        }
      }

    private:
      ros::NodeHandle nh_;
  };
}
#endif
