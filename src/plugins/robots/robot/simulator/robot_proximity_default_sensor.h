/**
 * @file <argos3/plugins/robots/prototype/simulator/robot_proximity_default_sensor.h>
 *
 * @author Michael Allwright - <allsey@gmail.com>
 */

#ifndef ROBOT_PROXIMITY_DEFAULT_SENSOR_H
#define ROBOT_PROXIMITY_DEFAULT_SENSOR_H

#include <string>
#include <map>

namespace argos {
   class CRobotProximityDefaultSensor;
}

#include <argos3/plugins/robots/robot/control_interface/ci_robot_proximity_sensor.h>
#include <argos3/plugins/simulator/sensors/proximity_default_sensor.h>

namespace argos {

   class CRobotProximityDefaultSensor : public CCI_RobotProximitySensor,
                                        public CSimulatedSensor {

   public:

      CRobotProximityDefaultSensor();

      virtual ~CRobotProximityDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

   private:

      CProximityDefaultSensor* m_pcProximityImpl;

   };

}

#endif
