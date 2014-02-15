/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/prototype_radios_default_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_RADIOS_DEFAULT_SENSOR_H
#define PROTOTYPE_RADIOS_DEFAULT_SENSOR_H

namespace argos {
   class CPrototypeRadiosDefaultSensor;
}

#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_radios_sensor.h>
#include <argos3/plugins/robots/prototype/simulator/entities/radio_equipped_entity.h>

namespace argos {

   class CPrototypeRadiosDefaultSensor : public CSimulatedSensor,
                                         public CCI_PrototypeRadiosSensor {

   public:

      CPrototypeRadiosDefaultSensor();

      virtual ~CPrototypeRadiosDefaultSensor() {}

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
