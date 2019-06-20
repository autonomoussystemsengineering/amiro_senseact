#ifndef TYPES_H_
#define TYPES_H_

/*! \brief Types to use if you work with the AMiRo platform
 *
 *  This header contains types which has to be used
 *  if they suit the problem.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly in µ iff the variable 
 *  is of type integer, unless it is explicitly named in
 *  the variable.
 *  All physical constants (therefore all values with a
 *  physical unit) are implicitly without prefix (e.g. µ)
 *  iff the variable is of type float, unless it is
 *  explicitly named in the variable.
 *  For the types, the SI prefix is used, iff the variable
 *  is of type float and therefor in SI units.
 */

namespace types {
  
  /**
   * A structure to represent the position and orientation of the robot
   */
  struct position { 
    /*@{*/
    int x;   /**< the x coordinate in µm */
    int y;   /**< the y coordinate in µm */
    int z;   /**< the z coordinate in µm */
    int f_x; /**< the f_x orientation in µrad */
    int f_y; /**< the f_y orientation in µrad */
    int f_z; /**< the f_z orientation in µrad */
    /*@}*/
  };
                     
  /**
   * A structure to represent the position and orientation of the robot
   */
  struct positionSI { 
    /*@{*/
    float x;   /**< the x coordinate in m */
    float y;   /**< the y coordinate in m */
    float z;   /**< the z coordinate in m */
    float f_x; /**< the f_x coordinate in rad */
    float f_y; /**< the f_y coordinate in rad */
    float f_z; /**< the f_z coordinate in rad */
    /*@}*/
  };
                       
  /**
   * A structure to represent the kinematics of the robot
   */
  struct kinematic { 
    /*@{*/
    int x;   /**< the x velocity in µm/s */
    int y;   /**< the y velocity in µm/s */
    int z;   /**< the z velocity in µm/s */
    int w_x; /**< the w_x angular velocity in µrad/s */
    int w_y; /**< the w_y angular velocity in µrad/s */
    int w_z; /**< the w_z angular velocity in µrad/s */
    /*@}*/
  };
                     
  /**
   * A structure to represent the kinematics of the robot
   */
  struct kinematicSI { 
    /*@{*/
    float x;   /**< the x velovity in m/s */
    float y;   /**< the y velocity in m/s */
    float z;   /**< the z velocity in m/s */
    float f_x; /**< the f_x angular velocity in rad/s */
    float f_y; /**< the f_y angular velocity in rad/s */
    float f_z; /**< the f_z angular velocity in rad/s */
    /*@}*/
  };
  
}

#endif /* TYPES_ */
