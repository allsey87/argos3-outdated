/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/prototype_joints_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


#ifndef PROTOTYPE_JOINTS_DEFAULT_SENSOR_H
#define PROTOTYPE_JOINTS_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CPrototypeJointsDefaultSensor;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_sensor.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/joint_equipped_entity.h>
#include <argos3/core/simulator/sensor.h>

namespace argos {

   class CPrototypeJointsDefaultSensor : public CSimulatedSensor,
                                         public CCI_PrototypeJointsSensor {

   public:
      class CSimulatedJointSensor : public CJointSensor {
      public:
         CSimulatedJointSensor(CJointEntity* pc_sensor_joint) :
            CJointSensor(pc_sensor_joint->GetId(), pc_sensor_joint->IsEnabled()),
            m_pcSensorJoint(pc_sensor_joint) {}
         
         CJointEntity& GetJoint() {
            return *m_pcSensorJoint;
         }
         
         void Reset() {
            m_cJointRotationReading = CQuaternion();
            m_cJointTranslationReading = CVector3();
            m_bSensorEnabled = false;
         }

         void Update() {
            m_cJointRotationReading = m_pcSensorJoint->GetJointRotation();
            m_cJointTranslationReading = m_pcSensorJoint->GetJointTranslation();
         }
      private:
         CJointEntity* m_pcSensorJoint;
      };
      
   public:

      CPrototypeJointsDefaultSensor();
      virtual ~CPrototypeJointsDefaultSensor() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual CJointSensor* GetJointSensor(std::string str_joint_id);

      virtual std::vector<CJointSensor*> GetAllJointSensors();

      virtual void Update();

      virtual void Reset();

   private:

      std::vector<CSimulatedJointSensor*> m_vecSensors;

      CJointEquippedEntity* m_pcJointEquippedEntity;

   };
}

#endif
