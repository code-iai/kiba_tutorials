#ifndef KIBA_CONTROL_UTILS_H
#define KIBA_CONTROL_UTILS_H

#include <ros/ros.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace kiba_control
{
  template<class T>
  inline T readParam(const ros::NodeHandle& nh, const std::string& name)
  {
    T param;
    if(!nh.getParam(name, param))
      throw std::runtime_error("Could not find parameter '" + name +
          "' in namespace '" + nh.getNamespace() + "'.");
    return param;
  }

  inline urdf::Model read_robot_model(const ros::NodeHandle& nh)
  {
    std::string robot_description = readParam<std::string>(nh, "/robot_description");

    urdf::Model robot_model;
    if (!robot_model.initString(robot_description))
      throw std::runtime_error("Could not parse robot description into URDF model.");

    return robot_model;
  }

  inline KDL::Chain extract_chain(const ros::NodeHandle& nh, const urdf::Model& robot_model,
      const std::string& root_frame_name, const std::string& tip_frame_name)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
      throw std::runtime_error("Could not extract KDL::Tree from URDF model.");
    KDL::Chain chain;
    if (!tree.getChain(root_frame_name, tip_frame_name, chain))
      throw std::runtime_error("Could not extract KDL::Chain with root '" + 
          root_frame_name + "' and tip '" + tip_frame_name + "'.");
    return chain;
  }
}

#endif
