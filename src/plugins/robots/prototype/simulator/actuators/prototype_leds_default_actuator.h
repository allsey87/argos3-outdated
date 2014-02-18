/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype/actuators/prototype_leds_default_actuator.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


#ifndef PROTOTYPE_LEDS_DEFAULT_ACTUATOR_H
#define PROTOTYPE_LEDS_DEFAULT_ACTUATOR_H

#include <string>
#include <map>

namespace argos {
   class CPrototypeLEDsDefaultActuator;
}

#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_led_equipped_entity.h>
#include <argos3/core/simulator/actuator.h>

namespace argos {

   class CPrototypeLEDsDefaultActuator : public CSimulatedActuator,
                                         public CCI_LEDsActuator {

   public:

      CPrototypeLEDsDefaultActuator();

      virtual ~CPrototypeLEDsDefaultActuator() {}

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Update();
      virtual void Reset();
      virtual void Destroy();

   private:

      CPrototypeLEDEquippedEntity* m_pcLEDEquippedEntity;

   };

}

#endif
