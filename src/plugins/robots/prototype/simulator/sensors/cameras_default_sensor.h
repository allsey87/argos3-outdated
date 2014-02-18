/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_DEFAULT_SENSOR_H
#define CAMERAS_DEFAULT_SENSOR_H

namespace argos {
   class CCamerasDefaultSensor;
   //   class CCamerasSensorSimulatedAlgorithm;
   class CCameraEquippedEntity;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor.h>

#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithm.h>

namespace argos {

   class CCamerasDefaultSensor : public CSimulatedSensor,
                                 public CCI_CamerasSensor {

   public:

      CCamerasDefaultSensor();

      virtual ~CCamerasDefaultSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual void Enable();

      virtual void Disable();


   public:

      const std::vector<CCamerasSensorSimulatedAlgorithm::SViewport>& GetViewports() const {
         return m_vecViewports;
      }

   protected:

      bool                               m_bEnabled;
      CCameraEquippedEntity*             m_pcCameraEquippedEntity;
      CControllableEntity*               m_pcControllableEntity;

      std::vector<CCamerasSensorSimulatedAlgorithm::SViewport> m_vecViewports;
      
      std::map<std::string, CCamerasSensorSimulatedAlgorithm::TMap, std::less<std::string> > m_mapSimulatedAlgorithms;
   };
}

#endif
