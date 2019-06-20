//#line 2 "/opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template"
// *********************************************************
//
// File autogenerated for the sick_tim package
// by the dynamic_reconfigure package.
// Please do not edit.
//
// ********************************************************/

#ifndef __sick_tim__SICKTIMCONFIG_H__
#define __sick_tim__SICKTIMCONFIG_H__

#include <string>

namespace sick_tim
{
  class SickTimConfigStatics;

  class SickTimConfig
  {
  public:
      double min_ang = -2.35619449019;
      double max_ang = 2.35619449019;
      bool intensity = false;
      int skip = 0;
      std::string frame_id = "";
      double time_offset = 0.0;
  };
}

#endif // __SICKTIMRECONFIGURATOR_H__