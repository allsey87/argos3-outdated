/**
 * @file <argos3/plugins/robots/robot/simulator/robot_joints_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "robot_joints_default_actuator.h"

namespace argos {

   /****************************************/
   /****************************************/

   CRobotJointsDefaultActuator::CRobotJointsDefaultActuator() :
      m_pcJointEquippedEntity(NULL) {
      /* Assign a vector for each axis to store the actuators for the joints */
      m_vecActuators.assign(6, std::vector<CSimulatedJointActuator>());
   }

   /****************************************/
   /****************************************/

   void CRobotJointsDefaultActuator::SetRobot(CComposableEntity& c_entity) {
      m_pcJointEquippedEntity = &(c_entity.GetComponent<CJointEquippedEntity>("joints"));
      m_pcJointEquippedEntity->SetCanBeEnabledIfDisabled(true);
      m_pcJointEquippedEntity->Enable();

      /* create a CSimulatedJointActuator for each linear and angular axis of each joint */
      for(CJointEntity::TList::iterator itJoint =  m_pcJointEquippedEntity->GetAllJoints().begin();
          itJoint != m_pcJointEquippedEntity->GetAllJoints().end();
          ++itJoint) {
         m_vecActuators[LINEAR_X].push_back(CSimulatedJointActuator(*itJoint, LINEAR_X));
         m_vecActuators[LINEAR_Y].push_back(CSimulatedJointActuator(*itJoint, LINEAR_Y));
         m_vecActuators[LINEAR_Z].push_back(CSimulatedJointActuator(*itJoint, LINEAR_Z));
         m_vecActuators[ANGULAR_X].push_back(CSimulatedJointActuator(*itJoint, ANGULAR_X));
         m_vecActuators[ANGULAR_Y].push_back(CSimulatedJointActuator(*itJoint, ANGULAR_Y));
         m_vecActuators[ANGULAR_Z].push_back(CSimulatedJointActuator(*itJoint, ANGULAR_Z));
      }
   }

   /****************************************/
   /****************************************/

   CRobotJointsDefaultActuator::CJointActuator& CRobotJointsDefaultActuator::GetJointActuator(std::string str_joint_id, EActuatorAxis e_axis) {
      std::vector<CSimulatedJointActuator>::iterator itActuator;
      /* search the collection and locate the correct actuator using the joint id */
      for(itActuator = m_vecActuators[e_axis].begin();
          itActuator != m_vecActuators[e_axis].end();
          ++itActuator) {
         if(itActuator->GetJoint().GetId() == str_joint_id) {
            break;
         }
      }
      return *itActuator;
   }

   /****************************************/
   /****************************************/

   void CRobotJointsDefaultActuator::Update() {
      for(std::vector<std::vector<CSimulatedJointActuator> >::iterator itAxis = m_vecActuators.begin();
          itAxis != m_vecActuators.end();
          ++itAxis) {
         for(std::vector<CSimulatedJointActuator>::iterator itActuator = itAxis->begin();
             itActuator != itAxis->end();
             ++itActuator) {
            switch(itActuator->GetAxis()) {
            case LINEAR_X:
               itActuator->GetJoint().SetActuatorParametersLinearX(itActuator->GetTargetVelocity(),
                                                        itActuator->GetEnabled());
               break;
            case LINEAR_Y:
               itActuator->GetJoint().SetActuatorParametersLinearY(itActuator->GetTargetVelocity(),
                                                        itActuator->GetEnabled());
               break;
            case LINEAR_Z:
               itActuator->GetJoint().SetActuatorParametersLinearZ(itActuator->GetTargetVelocity(),
                                                        itActuator->GetEnabled());
               break;
            case ANGULAR_X:
               itActuator->GetJoint().SetActuatorParametersAngularX(itActuator->GetTargetVelocity(),
                                                         itActuator->GetEnabled());
               break;
            case ANGULAR_Y:
               itActuator->GetJoint().SetActuatorParametersAngularY(itActuator->GetTargetVelocity(),
                                                         itActuator->GetEnabled());
               break;
            case ANGULAR_Z:
               itActuator->GetJoint().SetActuatorParametersAngularZ(itActuator->GetTargetVelocity(),
                                                         itActuator->GetEnabled());
               break;
            }
         }
      }
   }

   /****************************************/
   /****************************************/

   void CRobotJointsDefaultActuator::Reset() {
     for(std::vector<std::vector<CSimulatedJointActuator> >::iterator itAxis = m_vecActuators.begin();
          itAxis != m_vecActuators.end();
          ++itAxis) {
         for(std::vector<CSimulatedJointActuator>::iterator itActuator = itAxis->begin();
             itActuator != itAxis->end();
             ++itActuator) {
            itActuator->SetEnabled(false);
            itActuator->SetTargetVelocity(0.0f);
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_ACTUATOR(CRobotJointsDefaultActuator,
                     "joints", "default",
                     "Michael Allwright [allsey87@gmail.com]",
                     "1.0",
                     "The robot joint actuator.",
                     "This actuator controls a specified joint of the robot entity. For a complete\n"
                     "description of its usage, refer to the ci_robot_joint_actuator\n"
                     "file.\n\n"
                     "REQUIRED XML CONFIGURATION\n\n"
                     "  <controllers>\n"
                     "    ...\n"
                     "    <my_controller ...>\n"
                     "      ...\n"
                     "      <actuators>\n"
                     "        ...\n"
                     "        <joints implementation=\"default\"/>\n"
                     "        ...\n"
                     "      </actuators>\n"
                     "      ...\n"
                     "    </my_controller>\n"
                     "    ...\n"
                     "  </controllers>\n\n"
                     "OPTIONAL XML CONFIGURATION\n\n"
                     "None for the time being.\n",
                     "Usable"
      );
}
