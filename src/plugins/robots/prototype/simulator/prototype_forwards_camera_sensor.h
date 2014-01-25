/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_forwards_camera_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_FORWARDS_CAMERA_SENSOR_H
#define PROTOTYPE_FORWARDS_CAMERA_SENSOR_H

namespace argos {
   class CPrototypeForwardsCameraSensor;
   class CForwardsCameraEquippedEntity;
   class CLEDEntity;
   class CControllableEntity;
   class CLEDCheckOperation;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_forwards_camera_sensor.h>

namespace argos {

   class CPrototypeForwardsCameraSensor : public CSimulatedSensor,
                                          public CCI_PrototypeForwardsCameraSensor {

   public:

      CPrototypeForwardsCameraSensor();

      virtual ~CPrototypeForwardsCameraSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual void Enable();

      virtual void Disable();

   protected:

      bool                                    m_bEnabled;
      CForwardsCameraEquippedEntity*          m_pcForwardsCamerasEntity;
      CControllableEntity*                    m_pcControllableEntity;
      CEmbodiedEntity*                        m_pcEmbodiedEntity;
      CPositionalIndex<CLEDEntity>*           m_pcLEDIndex;
      CPositionalIndex<CEmbodiedEntity>*      m_pcEmbodiedIndex;
      CLEDCheckOperation*                     m_pcOperation;
      Real                                    m_fDistanceNoiseStdDev;
      CRandom::CRNG*                          m_pcRNG;
      bool                                    m_bShowRays;

   };
}

#endif
