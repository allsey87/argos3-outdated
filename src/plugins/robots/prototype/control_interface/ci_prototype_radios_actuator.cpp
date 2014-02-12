/**
 * @file <argos3/plugins/robots/prototype/control_interface/ci_prototype_radio_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_prototype_radios_actuator.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   /*
    * The stack must have two values in this order:
    * 1. radio identifier (a string)
    * 2. a list of bytes that compose the message (a table)
    */
   int LuaSendMessage(lua_State* pt_lua_state) {
      /* Check parameters */
      if(lua_gettop(pt_lua_state) != 2) {
         return luaL_error(pt_lua_state, "robot.radios.set_current() expects 2 arguments");
      }
      luaL_checktype(pt_lua_state, 1, LUA_TSTRING);
      luaL_checktype(pt_lua_state, 2, LUA_TTABLE);
      /* Get a reference to the specified radio */
      std::string strRadioId = lua_tostring(pt_lua_state, 1);
      CCI_PrototypeRadiosActuator* pcTargetActuator =
         CLuaUtility::GetDeviceInstance<CCI_PrototypeRadiosActuator>(pt_lua_state, "radios");
      UInt32 unRadioIdx;
      for(unRadioIdx = 0;
          unRadioIdx < pcTargetActuator->GetDescriptors().size();
          ++unRadioIdx) {
         if(pcTargetActuator->GetDescriptors()[unRadioIdx].Id == strRadioId)
            break;
      }
      if(unRadioIdx < pcTargetActuator->GetDescriptors().size()) {
         CByteArray cBuffer(lua_objlen(pt_lua_state, -1));
         for(size_t i = 0; i < cBuffer.Size(); ++i) {
            lua_pushnumber(pt_lua_state, i+1);
            lua_gettable(pt_lua_state, -2);
            if(lua_type(pt_lua_state, -1) == LUA_TNUMBER) {
               cBuffer[i] = static_cast<UInt8>(lua_tonumber(pt_lua_state, -1));
               lua_pop(pt_lua_state, 1);
            }
            else {
               return luaL_error(pt_lua_state, "element #%d of the array is not a number", i+1);
            }
         }
         pcTargetActuator->GetConfigurations()[unRadioIdx].TxData.push_back(cBuffer);
      }
      else {
         /* Radio not found */
         return luaL_error(pt_lua_state, "cannot find specified radio identifier");
      }
      return 0;
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA
   void CCI_PrototypeRadiosActuator::CreateLuaState(lua_State* pt_lua_state) {
      CLuaUtility::StartTable(pt_lua_state, "radios");
      CLuaUtility::AddToTable(pt_lua_state, "_instance", this);
      CLuaUtility::AddToTable(pt_lua_state, "send", &LuaSendMessage);
      CLuaUtility::EndTable(pt_lua_state);
   }
#endif


}
