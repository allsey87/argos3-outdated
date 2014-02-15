/**
 * @file <argos3/plugins/robots/prototype/simulator/actuators/prototype_electromagnets_default_actuator.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


#ifndef PROTOTYPE_ELECTROMAGNETS_DEFAULT_ACTUATOR_H
#define PROTOTYPE_ELECTROMAGNETS_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CPrototypeElectromagnetsDefaultActuator;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_electromagnets_actuator.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/electromagnet_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CPrototypeElectromagnetsDefaultActuator : public CSimulatedActuator,
                                                   public CCI_PrototypeElectromagnetsActuator {

   public:

      CPrototypeElectromagnetsDefaultActuator();
      virtual ~CPrototypeElectromagnetsDefaultActuator() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Update();

      virtual void Reset();

   private:

      CElectromagnetEquippedEntity* m_pcElectromagnetEquippedEntity;
      CControllableEntity* m_pcControllableEntity;

      std::vector<std::string> m_vecElectromagneticBodyIds;
      std::vector<UInt32> m_vecElectromagnetIndices;

      bool m_bShowPoles;

   };
}

#endif
