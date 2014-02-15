/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/camera_sensor_leddetector_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERA_SENSOR_LEDDETECTOR_ALGORITHM_H
#define CAMERA_SENSOR_LEDDETECTOR_ALGORITHM_H

namespace argos {
	class CCameraSensorLEDDetectorAlgorithm;
}

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>


namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    */
   class CCameraSensorLEDDetectorAlgorithm : public CCameraSensorAlgorithm,
                                             public CPositionalIndex<CLEDEntity>::COperation {
      
   public:

      struct SObservation {
         /* Color */
         CColor Color;
         /* Coordinates in image */
         UInt32 HorizontalIndex;
         UInt32 VerticalIndex;
         /**
          * Constructor
          */
         SObservation() :
            Color(CColor::BLACK),
            HorizontalIndex(0),
            VerticalIndex(0) {}
         /**
          * Constructor with parameters
          * @param c_color Observation color
          * @param un_horizontal_index horizontal index
          * @param un_vertical_index vertical index
          */
         SObservation(const CColor& c_color,
               UInt32 un_horizontal_index,
               UInt32 un_vertical_index) :
            Color(c_color),
            HorizontalIndex(un_horizontal_index),
            VerticalIndex(un_vertical_index) {}
         /**
          * Vector of observations.
          */
         typedef std::vector<SObservation> TList;
      };
      
      CCameraSensorLEDDetectorAlgorithm() {}

      virtual ~CCameraSensorLEDDetectorAlgorithm() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void Destroy() {}
  
      virtual void Update();
      
      virtual bool operator()(CLEDEntity& c_led);

   protected:

      bool m_bShowRays;

      

      CRay3 m_cOcclusionCheckRay;
      CControllableEntity& m_cControllableEntity;
      SEmbodiedEntityIntersectionItem m_sIntersectionItem;

      CPrototypeCamerasSensor::SReading* m_psReading;
      CPrototypeCamerasSensor::SViewport m_sViewport;

      CVector3    m_cCameraPosition;
      CVector3    m_cAttachedBodyPosition;
      CQuaternion m_cAttachedBodyOrientation;
      CVector3    m_cCameraPositionOffset;
      CQuaternion m_cCameraOrientationOffset;
      CRadians    m_cCameraRoll;

      UInt32 m_unHorizontalResolution;
      UInt32 m_unVerticalResolution;
  
   };
}         

