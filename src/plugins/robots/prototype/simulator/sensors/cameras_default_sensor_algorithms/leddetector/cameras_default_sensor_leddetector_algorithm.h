/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor_algorithms/cameras_default_sensor_leddetector_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_DEFAULT_SENSOR_LEDDETECTOR_ALGORITHM_H
#define CAMERAS_DEFAULT_SENSOR_LEDDETECTOR_ALGORITHM_H

namespace argos {
	class CCamerasDefaultSensorLEDDetectorAlgorithm;
}

#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor_algorithm.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/ray3.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_entity.h>


namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    */
   class CCamerasDefaultSensorLEDDetectorAlgorithm : public CCamerasDefaultSensorAlgorithm,
                                                     public CCI_CamerasSensorLEDDetectorAlgorithm,
                                                     public CPositionalIndex<CLEDEntity>::COperation {
      
   public:

      CCamerasDefaultSensorLEDDetectorAlgorithm() {}

      virtual ~CCamerasDefaultSensorLEDDetectorAlgorithm() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void Destroy() {}
  
      virtual void Update();

      virtual void SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index);

      virtual void SetViewport(const CCamerasDefaultSensor::SViewport& s_viewport) {
         m_sViewport = s_viewport;
      }

      virtual bool operator()(CLEDEntity& c_led);

   protected:

      CCameraEquippedEntity*             m_pcCameraEquippedEntity;
      CControllableEntity*               m_pcControllableEntity;
      UInt32                             m_unCameraIndex;
      bool                               m_bShowRays;
      CRay3                              m_cOcclusionCheckRay;
      Real                               m_fDistanceNoiseStdDev;
      CRandom::CRNG*                     m_pcRNG;
      CPositionalIndex<CLEDEntity>*      m_pcLEDIndex;

      SEmbodiedEntityIntersectionItem    m_sIntersectionItem;
 
      //CPrototypeCamerasSensor::SReading* m_psReading;
      CCamerasDefaultSensor::SViewport   m_sViewport;

      CVector3                           m_cCameraPosition;
      CVector3                           m_cAttachedBodyPosition;
      CQuaternion                        m_cAttachedBodyOrientation;
      CVector3                           m_cCameraPositionOffset;
      CQuaternion                        m_cCameraOrientationOffset;
      CRadians                           m_cCameraRoll;

      UInt32                             m_unHorizontalResolution;
      UInt32                             m_unVerticalResolution;
  
   };
}         

#endif
