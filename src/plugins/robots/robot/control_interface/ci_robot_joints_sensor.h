/**
 * @file <argos3/plugins/robots/robot/control_interface/ci_robot_joints_sensor.h>
 *
 * @brief This file provides the common interface definition of the joints sensor.
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CCI_ROBOT_JOINTS_SENSOR_H
#define CCI_ROBOT_JOINTS_SENSOR_H

namespace argos {
   class CCI_RobotJointsSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

   class CCI_RobotJointsSensor : virtual public CCI_Sensor {

   public:

      class CJointSensor {
      public:
         CJointSensor(const std::string& str_id, bool b_enabled = false) :
            m_bSensorEnabled(b_enabled),
            m_strSensorId(str_id) {}
         
         void GetReading(CVector3& c_translation, CQuaternion& c_rotation) const {
            c_rotation = m_cJointRotationReading;
            c_translation = m_cJointTranslationReading;
         }

         void GetReading(CVector3& c_translation) const {
            c_translation = m_cJointTranslationReading;
         }

         void GetReading(CQuaternion& c_rotation) const {
            c_rotation = m_cJointRotationReading;
         }

         //@todo void GetRange() - implement and add to Lua interface
         
         void SetEnabled(bool b_enabled) {
            m_bSensorEnabled = b_enabled;
         }
   
         bool GetEnabled() const {
            return m_bSensorEnabled;
         }

         const std::string& GetId() const {
            return m_strSensorId;
         }
      protected:
         CQuaternion m_cJointRotationReading;
         CVector3 m_cJointTranslationReading;
         bool m_bSensorEnabled;
         std::string m_strSensorId;
      };

   public:

      /**
       * Destructor.
       */
      virtual ~CCI_RobotJointsSensor() {}
      
      virtual CJointSensor* GetJointSensor(std::string str_joint_id) = 0;

      virtual std::vector<CJointSensor*> GetAllJointSensors() = 0;

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);

      void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   };
}

#endif
