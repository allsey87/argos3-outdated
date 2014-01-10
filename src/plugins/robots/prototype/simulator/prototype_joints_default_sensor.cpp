/**
 * @file <argos3/plugins/robots/prototype/simulator/robot_joints_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_joints_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeJointsDefaultSensor::CPrototypeJointsDefaultSensor() :
      m_pcJointEquippedEntity(NULL) {
   }

   /****************************************/
   /****************************************/

   void CPrototypeJointsDefaultSensor::SetRobot(CComposableEntity& c_entity) {
      m_pcJointEquippedEntity = &(c_entity.GetComponent<CJointEquippedEntity>("joints"));
      m_pcJointEquippedEntity->SetCanBeEnabledIfDisabled(true);
      m_pcJointEquippedEntity->Enable();

      /* create a CSimulatedJointSensor for each linear and angular axis of each joint */
      for(CJointEntity::TList::iterator itJoint =  m_pcJointEquippedEntity->GetAllJoints().begin();
          itJoint != m_pcJointEquippedEntity->GetAllJoints().end();
          ++itJoint) {
         m_vecSensors.push_back(new CSimulatedJointSensor(*itJoint));
      }
   }

   /****************************************/
   /****************************************/

   CPrototypeJointsDefaultSensor::CJointSensor* CPrototypeJointsDefaultSensor::GetJointSensor(std::string str_joint_id) {
      std::vector<CSimulatedJointSensor*>::iterator itSensor;
      /* search the collection and locate the correct sensor using the joint id */
      for(itSensor = m_vecSensors.begin();
          itSensor != m_vecSensors.end();
          ++itSensor) {
         if((*itSensor)->GetJoint().GetId() == str_joint_id) {
            break;
         }
      }
      return *itSensor;
   }


   /****************************************/
   /****************************************/

   std::vector<CPrototypeJointsDefaultSensor::CJointSensor*> CPrototypeJointsDefaultSensor::GetAllJointSensors() {
      std::vector<CJointSensor*> vecSensors;
      for(std::vector<CSimulatedJointSensor*>::iterator itSensor = m_vecSensors.begin();
          itSensor != m_vecSensors.end();
          ++itSensor) {
         vecSensors.push_back(*itSensor);
      }
      return vecSensors;
   }

   /****************************************/
   /****************************************/

   void CPrototypeJointsDefaultSensor::Update() {
      std::vector<CSimulatedJointSensor*>::iterator itSensor;
      /* search the collection and locate the correct sensor using the joint id */
      for(itSensor = m_vecSensors.begin();
          itSensor != m_vecSensors.end();
          ++itSensor) {
         (*itSensor)->Update();
      }
   }

   /****************************************/
   /****************************************/

   void CPrototypeJointsDefaultSensor::Reset() {
      std::vector<CSimulatedJointSensor*>::iterator itSensor;
      /* search the collection and locate the correct sensor using the joint id */
      for(itSensor = m_vecSensors.begin();
          itSensor != m_vecSensors.end();
          ++itSensor) {
         (*itSensor)->Reset();
      } 
   }

   /****************************************/
   /****************************************/

   REGISTER_SENSOR(CPrototypeJointsDefaultSensor,
                     "joints", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "The prototype joint sensor.",
                     "This sensor controls a specified joint of the prototype entity. For a complete\n"
                     "description of its usage, refer to the ci_prototype_joint_sensor\n"
                     "file.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <sensors>\n"
                     "        ...\n"
                     "        <joints implementation=\"default\"/>\n"
                     "        ...\n"
                     "      </sensors>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "None for the time being.\n",
                     "Under Development"
      );
}
