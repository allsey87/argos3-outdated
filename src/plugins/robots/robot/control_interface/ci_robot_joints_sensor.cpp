/**
 * @file <argos3/plugins/robots/robotic/control_interface/ci_robot_joints_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "ci_robot_joints_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

   /****************************************/
   /****************************************/


#ifdef ARGOS_WITH_LUA


   void CCI_RobotJointsSensor::CreateLuaState(lua_State* pt_lua_state) {     
   
      const std::vector<CJointSensor*>& vecSensors = GetAllJointSensors();
      CVector3 cTranslationReading;
      CQuaternion cRotationReading;
      CRadians cRotationReadingZ, cRotationReadingY, cRotationReadingX;

      
      CLuaUtility::OpenRobotStateTable(pt_lua_state, "joints");
      for(std::vector<CJointSensor*>::const_iterator itSensor = vecSensors.begin();
       itSensor != vecSensors.end();
          ++itSensor) {
         
         (*itSensor)->GetReading(cTranslationReading, cRotationReading);
         cRotationReading.ToEulerAngles(cRotationReadingZ, cRotationReadingY, cRotationReadingX);
         
         CLuaUtility::StartTable(pt_lua_state, (*itSensor)->GetId());
         CLuaUtility::AddToTable(pt_lua_state, "translation_x", cTranslationReading.GetX());
         CLuaUtility::AddToTable(pt_lua_state, "translation_y", cTranslationReading.GetY());
         CLuaUtility::AddToTable(pt_lua_state, "translation_z", cTranslationReading.GetZ());
         CLuaUtility::AddToTable(pt_lua_state, "rotation_z", cRotationReadingZ);
         CLuaUtility::AddToTable(pt_lua_state, "rotation_y", cRotationReadingY);
         CLuaUtility::AddToTable(pt_lua_state, "rotation_x", cRotationReadingX);
         
         CLuaUtility::EndTable(pt_lua_state);
      }
      CLuaUtility::CloseRobotStateTable(pt_lua_state);
   }
#endif

   /****************************************/
   /****************************************/

#ifdef ARGOS_WITH_LUA

   void CCI_RobotJointsSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
      const std::vector<CJointSensor*>& vecSensors = GetAllJointSensors();

      CVector3 cTranslationReading;
      CQuaternion cRotationReading;
      CRadians cRotationReadingZ, cRotationReadingY, cRotationReadingX;   

      lua_getfield(pt_lua_state, -1, "joints");
      // Overwrite the table with the new messages
      for(std::vector<CJointSensor*>::const_iterator itSensor = vecSensors.begin();
          itSensor != vecSensors.end();
          ++itSensor) {

         (*itSensor)->GetReading(cTranslationReading, cRotationReading);
         cRotationReading.ToEulerAngles(cRotationReadingZ, cRotationReadingY, cRotationReadingX);
         
         CLuaUtility::StartTable(pt_lua_state, (*itSensor)->GetId());
         CLuaUtility::AddToTable(pt_lua_state, "translation_x", cTranslationReading.GetX());
         CLuaUtility::AddToTable(pt_lua_state, "translation_y", cTranslationReading.GetY());
         CLuaUtility::AddToTable(pt_lua_state, "translation_z", cTranslationReading.GetZ());
         CLuaUtility::AddToTable(pt_lua_state, "rotation_z", cRotationReadingZ);
         CLuaUtility::AddToTable(pt_lua_state, "rotation_y", cRotationReadingY);
         CLuaUtility::AddToTable(pt_lua_state, "rotation_x", cRotationReadingX);
      
         CLuaUtility::EndTable(pt_lua_state);
      }
      lua_pop(pt_lua_state, 1);
   }
#endif

   /****************************************/
   /****************************************/

}
