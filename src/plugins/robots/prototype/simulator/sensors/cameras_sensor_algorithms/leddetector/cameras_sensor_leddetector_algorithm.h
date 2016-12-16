/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/cameras_sensor_leddetector_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_SENSOR_LEDDETECTOR_ALGORITHM_H
#define CAMERAS_SENSOR_LEDDETECTOR_ALGORITHM_H

namespace argos {
	class CCamerasSensorLEDDetectorAlgorithm;
}

#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithm.h>
#include <argos3/plugins/simulator/entities/led_entity.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>
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
   class CCamerasSensorLEDDetectorAlgorithm : public CCamerasSensorSimulatedAlgorithm,
                                              public CCI_CamerasSensorLEDDetectorAlgorithm,
                                              public CPositionalIndex<CLEDEntity>::COperation {
      
   public:

      CCamerasSensorLEDDetectorAlgorithm();

      virtual ~CCamerasSensorLEDDetectorAlgorithm() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}

      virtual void Destroy() {}
  
      virtual void Update();

      virtual bool operator()(CLEDEntity& c_led);

   protected:
      CControllableEntity*               m_pcControllableEntity;
      bool                               m_bShowRays;
      CRay3                              m_cOcclusionCheckRay;
      Real                               m_fDistanceNoiseStdDev;
      CRandom::CRNG*                     m_pcRNG;
      CPositionalIndex<CLEDEntity>*      m_pcLEDIndex;
      SEmbodiedEntityIntersectionItem    m_sIntersectionItem;
   };
}         

#endif
