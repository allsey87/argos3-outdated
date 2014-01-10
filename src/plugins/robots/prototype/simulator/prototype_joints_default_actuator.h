/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_joints_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


#ifndef PROTOTYPE_JOINTS_DEFAULT_ACTUATOR_H
#define PROTOTYPE_JOINTS_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CPrototypeJointsDefaultActuator;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_joints_actuator.h>
#include <argos3/plugins/robots/prototype/simulator/prototype_entity.h>
#include <argos3/plugins/robots/prototype/simulator/joint_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CPrototypeJointsDefaultActuator : public CSimulatedActuator,
                                       public CCI_PrototypeJointsActuator {

   public:
      class CSimulatedJointActuator : public CJointActuator {
      public:
         CSimulatedJointActuator(CJointEntity* pc_actuator_joint,
                                 EActuatorAxis e_actuator_axis) :
            m_pcActuatorJoint(pc_actuator_joint),
            m_eActuatorAxis(e_actuator_axis) {}
         EActuatorAxis GetAxis() {
            return m_eActuatorAxis;
         }
         CJointEntity& GetJoint() {
            return *m_pcActuatorJoint;
         }
      private:
         CJointEntity* m_pcActuatorJoint;
         EActuatorAxis m_eActuatorAxis;
      };
      
   public:

      CPrototypeJointsDefaultActuator();
      virtual ~CPrototypeJointsDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual CJointActuator& GetJointActuator(std::string str_joint_id, EActuatorAxis e_axis);

      virtual void Update();

      virtual void Reset();

   private:

      std::vector<std::vector<CSimulatedJointActuator> > m_vecActuators;

      CJointEquippedEntity* m_pcJointEquippedEntity;

   };
}

#endif
