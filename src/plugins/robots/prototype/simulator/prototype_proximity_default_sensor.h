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
   class CPrototypeProximitySensorEquippedEntity;
   //   class CPrototypeProximityDefaultSensor;
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

      virtual Real CalculateReading(Real f_distance);

   protected:

      /** Reference to embodied entity associated to this sensor */
      CEmbodiedEntity* m_pcEmbodiedEntity;

      /** Reference to proximity sensor equipped entity associated to this sensor */
      CPrototypeProximitySensorEquippedEntity* m_pcPrototypeProximityEntity;

      /** Reference to controllable entity associated to this sensor */
      CControllableEntity* m_pcControllableEntity;

      /** Flag to show rays in the simulator */
      bool m_bShowRays;

      /** Random number generator */
      CRandom::CRNG* m_pcRNG;

      /** Whether to add noise or not */
      bool m_bAddNoise;

      /** Noise range */
      CRange<Real> m_cNoiseRange;

      /** Reference to the space */
      //CSpace& m_cSpace;
   };
}

#endif
