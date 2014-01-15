/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_colored_blob_forwards_camera_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef PROTOTYPE_COLORED_BLOB_FORWARDS_CAMERA_SENSOR_H
#define PROTOTYPE_COLORED_BLOB_FORWARDS_CAMERA_SENSOR_H

namespace argos {
   class CPrototypeColoredBlobForwardsCameraSensor;
   class CPrototypeForwardsCameraEquippedEntity;
   class CLEDEntity;
   class CControllableEntity;
   class CLEDCheckOperation;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_prototype_colored_blob_forwards_camera_sensor.h>

namespace argos {

   class CPrototypeColoredBlobForwardsCameraSensor : public CSimulatedSensor,
                                                     public CCI_PrototypeColoredBlobForwardsCameraSensor {

   public:

      CPrototypeColoredBlobForwardsCameraSensor();

      virtual ~CPrototypeColoredBlobForwardsCameraSensor();

      virtual void SetRobot(CComposableEntity& c_entity);

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Update();

      virtual void Reset();

      virtual void Destroy();

      virtual void Enable();

      virtual void Disable();

   protected:

      bool                                    m_bEnabled;
      CPrototypeForwardsCameraEquippedEntity* m_pcForwardsCamEntity;
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
