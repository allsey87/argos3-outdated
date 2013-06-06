/**
 * @file <argos3/plugins/robots/robot/simulator/robot_joints_default_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "robot_joints_default_sensor.h"

namespace argos {

   /****************************************/
   /****************************************/

   CRobotJointsDefaultSensor::CRobotJointsDefaultSensor() :
      m_pcJointEquippedEntity(NULL) {
   }

   /****************************************/
   /****************************************/

   void CRobotJointsDefaultSensor::SetRobot(CComposableEntity& c_entity) {
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

   CRobotJointsDefaultSensor::CJointSensor* CRobotJointsDefaultSensor::GetJointSensor(std::string str_joint_id) {
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

   std::vector<CRobotJointsDefaultSensor::CJointSensor*> CRobotJointsDefaultSensor::GetAllJointSensors() {
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

   void CRobotJointsDefaultSensor::Update() {
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

   void CRobotJointsDefaultSensor::Reset() {
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

   REGISTER_SENSOR(CRobotJointsDefaultSensor,
                     "joints", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "The robot joint sensor.",
                     "This sensor controls a specified joint of the robot entity. For a complete\n"
                     "description of its usage, refer to the ci_robot_joint_sensor\n"
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
                     "Usable"
      );
}
