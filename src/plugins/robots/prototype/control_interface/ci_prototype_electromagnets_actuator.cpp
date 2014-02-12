/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_robot_electromagnet_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_prototype_electromagnets_actuator.h"

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
   int LuaSetElectromagnetCurrent(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 2) {
         return luaL_error(pt_lua_state, "robot.electromagnets.set_current() expects 2 arguments");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TSTRING);
      luaL_checktype(pt_lua_state, 2, LUA_TNUMBER);
      /* Get a reference to the specified electromagnet */
      std::string strElectromagnetId = lua_tostring(pt_lua_state, 1);
      CCI_PrototypeElectromagnetsActuator* pcTargetActuator =
         CLuaUtility::GetDeviceInstance<CCI_PrototypeElectromagnetsActuator>(pt_lua_state, "electromagnets");
      UInt32 unElectromagnetIdx;
      for(unElectromagnetIdx = 0;
          unElectromagnetIdx < pcTargetActuator->GetDescriptors().size();
          ++unElectromagnetIdx) {
         if(pcTargetActuator->GetDescriptors()[unElectromagnetIdx].Id == strElectromagnetId)
            break;
      }
      if(unElectromagnetIdx < pcTargetActuator->GetDescriptors().size()) {
         /* Perform action */
         pcTargetActuator->GetConfigurations()[unElectromagnetIdx].Current = lua_tonumber(pt_lua_state, 2);
      }
      else {
         /* Electromagnet not found */
         return luaL_error(pt_lua_state, "cannot find specified electromagnet identifier");
      }
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PrototypeElectromagnetsActuator::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::StartTable(pt_lua_state, "electromagnets");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "set_current", &LuaSetElectromagnetCurrent);
      CLuaUtility::EndTable(pt_lua_state);
   }
#endif


}
