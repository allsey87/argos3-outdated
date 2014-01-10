/**
 * @file <argos3/plugins/robots/prototype/simulator/prototype_proximity_default_sensor.h>
 *
 * @author Michael Allwright - <allsey@gmail.com>
 */

#ifndef PROTOTYPE_PROXIMITY_DEFAULT_SENSOR_H
#define PROTOTYPE_PROXIMITY_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CPrototypeProximityDefaultSensor;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_proximity_sensor.h>
#include <argos3/plugins/simulator/sensors/proximity_default_sensor.h>

namespace argos {

   class CPrototypeProximityDefaultSensor : public CCI_PrototypeProximitySensor,
                                            public CSimulatedSensor {

   public:

      CPrototypeProximityDefaultSensor();

      virtual ~CPrototypeProximityDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   private:

      CProximityDefaultSensor* m_pcProximityImpl;

   };

}

#endif
