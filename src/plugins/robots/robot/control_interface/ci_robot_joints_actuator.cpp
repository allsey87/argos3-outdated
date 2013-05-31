/**
 * @file <argos3/plugins/robots/robotic/control_interface/ci_robot_joints_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_robot_joints_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * The stack must have two values in this order:
    * 1. joint identifier (a string)
    * 2. axis identifier (a string)
    * 3. velocity (a number)
    */
   int LuaSetJointVelocity(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 3) {
         return luaL_error(pt_lua_state, "robot.joints.set_joint_velocity() expects 3 arguments");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TSTRING);
      luaL_checktype(pt_lua_state, 2, LUA_TSTRING);
      luaL_checktype(pt_lua_state, 3, LUA_TNUMBER);

      std::string strTargetAxis = lua_tostring(pt_lua_state, 2);
      CCI_RobotJointsActuator::EActuatorAxis eTargetAxis;

      if(strTargetAxis == "LINEAR_X") {
         eTargetAxis = CCI_RobotJointsActuator::LINEAR_X;
      }
      else if(strTargetAxis == "LINEAR_Y") {
         eTargetAxis = CCI_RobotJointsActuator::LINEAR_Y;
      }
      else if(strTargetAxis == "LINEAR_Z") {
         eTargetAxis = CCI_RobotJointsActuator::LINEAR_Z;
      }
      else if(strTargetAxis == "ANGULAR_X") {
         eTargetAxis = CCI_RobotJointsActuator::ANGULAR_X;
      }
      else if(strTargetAxis == "ANGULAR_Y") {
         eTargetAxis = CCI_RobotJointsActuator::ANGULAR_Y;
      }
      else if(strTargetAxis == "ANGULAR_Z") {
         eTargetAxis = CCI_RobotJointsActuator::ANGULAR_Z;
      }
      else {
         return luaL_error(pt_lua_state, "the second argument of robot.joints.set_joint_velocity() "
                                         "expects a string in the format of MODE_AXIS, e.g. "
                                         "\"LINEAR_X\" or \"ANGULAR_Z\"");
      }
      CCI_RobotJointsActuator::CJointActuator& m_cTargetJoint = 
         CLuaUtility::GetDeviceInstance<CCI_RobotJointsActuator>(pt_lua_state, "joints")->
         GetJointActuator(lua_tostring(pt_lua_state, 1), eTargetAxis);
      /* Perform action */
      m_cTargetJoint.SetTargetVelocity(lua_tonumber(pt_lua_state, 3));
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_RobotJointsActuator::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "joints");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "set_joint_velocity", &LuaSetJointVelocity);
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif


}
