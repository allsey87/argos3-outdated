/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_cameras_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_CAMERAS_SENSOR_H
#define PROTOTYPE_CAMERAS_SENSOR_H

namespace argos {
   class CPrototypeCamerasSensor;
   class CCameraEquippedEntity;
   class CLEDEntity;
   class CControllableEntity;
   class CLEDCheckOperation;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_cameras_sensor.h>

namespace argos {

   class CPrototypeCamerasSensor : public CSimulatedSensor,
                                   public CCI_PrototypeCamerasSensor {

   public:

      CPrototypeCamerasSensor();

      virtual ~CPrototypeCamerasSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual void Enable();

      virtual void Disable();

   public:

      struct SViewport {
         CVector3 Position;
         CQuaternion Orientation;
         CVector3 HalfExtents;
      };

      const std::vector<SViewport>& GetViewports() const {
         return m_vecViewports;
      }

   protected:

      bool                               m_bEnabled;
      CCameraEquippedEntity*             m_pcCamerasEntity;
      CControllableEntity*               m_pcControllableEntity;
      CEmbodiedEntity*                   m_pcEmbodiedEntity;
      CPositionalIndex<CLEDEntity>*      m_pcLEDIndex;
      CPositionalIndex<CEmbodiedEntity>* m_pcEmbodiedIndex;
      CLEDCheckOperation*                m_pcOperation;
      Real                               m_fDistanceNoiseStdDev;
      CRandom::CRNG*                     m_pcRNG;
      bool                               m_bShowRays;
      std::vector<SViewport>             m_vecViewports;

   };
}

#endif
