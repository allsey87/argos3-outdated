/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_DEFAULT_SENSOR_H
#define CAMERAS_DEFAULT_SENSOR_H

namespace argos {
   class CCamerasDefaultSensor;
   class CCamerasDefaultSensorAlgorithm;
   class CCameraEquippedEntity;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor.h>

//#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor_algorithm.h>

namespace argos {

   class CCamerasDefaultSensor : public CSimulatedSensor,
                                 public CCI_CamerasSensor {

   public:

      // struct SResult : public CCI_CamerasSensor::SCI_Result {
      //    // empty for the moment
      // };

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

      struct SViewport {
         CVector3 CameraLocation;
         CVector3 Position;
         CQuaternion Orientation;
         CVector3 HalfExtents;
      };

      const std::vector<SViewport>& GetViewports() const {
         return m_vecViewports;
      }

   protected:

      bool                               m_bEnabled;
      CCameraEquippedEntity*             m_pcCameraEquippedEntity;
      // CControllableEntity*               m_pcControllableEntity;
      std::vector<SViewport>             m_vecViewports;
   };
}

#endif
