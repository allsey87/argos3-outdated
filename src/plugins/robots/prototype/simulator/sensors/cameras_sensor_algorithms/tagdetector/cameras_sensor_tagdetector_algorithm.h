/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/tagdetector/cameras_sensor_tagdetector_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_SENSOR_TAGDETECTOR_ALGORITHM_H
#define CAMERAS_SENSOR_TAGDETECTOR_ALGORITHM_H

namespace argos {
	class CCamerasSensorTagDetectorAlgorithm;
}

#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithm.h>
#include <argos3/plugins/robots/prototype/simulator/entities/tag_entity.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_tagdetector_algorithm.h>
#include <argos3/core/simulator/space/positional_indices/positional_index.h>

#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/ray3.h>

#include <argos3/core/simulator/entity/embodied_entity.h>

#include <argos3/core/utility/math/matrix/matrix.h>
#include <argos3/core/utility/math/matrix/squarematrix.h>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>

namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    */
   class CCamerasSensorTagDetectorAlgorithm : public CCamerasSensorSimulatedAlgorithm,
                                              public CCI_CamerasSensorTagDetectorAlgorithm,
                                              public CPositionalIndex<CTagEntity>::COperation {
      
   public:

      CCamerasSensorTagDetectorAlgorithm();

      virtual ~CCamerasSensorTagDetectorAlgorithm() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void Destroy() {}
  
      virtual void Update();

      virtual void SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index);

      virtual void SetViewport(const SViewport& s_viewport) {
         m_sViewport = s_viewport;
      }

      virtual bool operator()(CTagEntity& c_tag);

      CVector2 Project(const CVector3& c_vector);

   protected:

      CCameraEquippedEntity*             m_pcCameraEquippedEntity;
      CControllableEntity*               m_pcControllableEntity;
      UInt32                             m_unCameraIndex;
      bool                               m_bShowRays;
      CRay3                              m_cOcclusionCheckRay;
      Real                               m_fDistanceNoiseStdDev;
      CRandom::CRNG*                     m_pcRNG;
      CPositionalIndex<CTagEntity>*      m_pcTagIndex;

      SEmbodiedEntityIntersectionItem    m_sIntersectionItem;
 
      SViewport                          m_sViewport;

      UInt32                             m_unHorizontalResolution;
      UInt32                             m_unVerticalResolution;

      CSquareMatrix<3>                   m_cCameraMatrix;
      CTransformationMatrix3             m_cCameraToBodyTransform;
      CMatrix<3,4>                       m_cCameraToWorldMatrix;

      const std::vector<CVector3> m_vecTagCornerOffsets = {
         { 0.5,  0.5, 0},
         {-0.5,  0.5, 0},
         {-0.5, -0.5, 0},
         { 0.5, -0.5, 0},
      };
   };
}         

#endif
