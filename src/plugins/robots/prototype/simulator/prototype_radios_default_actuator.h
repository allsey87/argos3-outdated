/*
 * @file <argos3/plugins/robots/prototype/simulator/prototype_radios_default_actuator.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_RADIOS_DEFAULT_ACTUATOR_H
#define PROTOTYPE_RADIOS_DEFAULT_ACTUATOR_H

namespace argos {
   class CPrototypeRadiosDefaultActuator;
}

#include <argos3/core/simulator/actuator.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_radios_actuator.h>
#include <argos3/plugins/robots/prototype/simulator/radio_equipped_entity.h>

namespace argos {

   class CPrototypeRadiosDefaultActuator : public CSimulatedActuator,
                                           public CCI_PrototypeRadiosActuator {

   public:

      CPrototypeRadiosDefaultActuator();

      virtual ~CPrototypeRadiosDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   protected:

      CRadioEquippedEntity* m_pcRadioEquippedEntity;
      CControllableEntity* m_pcControllableEntity;
   };
}

#endif
