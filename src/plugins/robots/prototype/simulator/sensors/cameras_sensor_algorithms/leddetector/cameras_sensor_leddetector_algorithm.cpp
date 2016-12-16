/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/cameras_sensor_leddetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


/*
 Reference: Robotics 2 Camera Calibration by Barbara Frank, Cyrill Stachniss, Giorgio Grisetti, Kai Arras, Wolfram Burgard
 Universitaet Freiburg
 http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/pdfs/rob2-08-camera-calibration.pdf

 discussion on culling objects https://forum.thegamecreators.com/thread/179559
*/

#include "cameras_sensor_leddetector_algorithm.h"

#include <argos3/core/simulator/simulator.h>

#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/plugins/simulator/entities/led_entity.h>

#include <argos3/core/utility/math/matrix/matrix.h>
#include <argos3/core/utility/math/matrix/squarematrix.h>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>

namespace argos {

   /****************************************/
   /****************************************/   

   CCamerasSensorLEDDetectorAlgorithm::CCamerasSensorLEDDetectorAlgorithm() :
      m_pcControllableEntity(NULL),
      m_bShowRays(false),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_pcLEDIndex(NULL) {}

   /****************************************/
   /****************************************/   
   
   void CCamerasSensorLEDDetectorAlgorithm::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_CamerasSensorAlgorithm::Init(t_tree);
         /* Show rays? */
         GetNodeAttributeOrDefault(t_tree, "show_rays", m_bShowRays, m_bShowRays);
         /* Parse noise */
         GetNodeAttributeOrDefault(t_tree, "noise_std_dev", m_fDistanceNoiseStdDev, m_fDistanceNoiseStdDev);
         if(m_fDistanceNoiseStdDev > 0.0f) {
            m_pcRNG = CRandom::CreateRNG("argos");
         }
         /* Get LED medium from id specified in the XML */
         std::string strMedium;
         GetNodeAttribute(t_tree, "medium", strMedium);
         m_pcLEDIndex = &(CSimulator::GetInstance().GetMedium<CLEDMedium>(strMedium).GetIndex());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the LED detector algorithm", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CCamerasSensorLEDDetectorAlgorithm::Update() {
      /* All occlusion rays start from the camera position */
      m_cOcclusionCheckRay.SetStart(m_psData->CameraLocation);
      /* Clear the old readings */ 
      m_tReadings.clear();
      m_vecCheckedRays.clear();
      m_pcLEDIndex->ForEntitiesInBoxRange(m_psData->BoundingBoxPosition,
                                          m_psData->BoundingBoxHalfExtents,
                                          *this);
   }

   /****************************************/
   /****************************************/

   bool CCamerasSensorLEDDetectorAlgorithm::operator()(CLEDEntity& c_led) {
      /* color check: black (off) LEDs are not considered */
      if(c_led.GetColor() == CColor::BLACK) {
         return true;
      }
      /* frustum check */
      const CVector3& cLedPosition = c_led.GetPosition();
      if(IsPointInsideFrustum(cLedPosition) == false) {
         return true;
      }
      /* ray check */
      m_cOcclusionCheckRay.SetEnd(cLedPosition);
      if(GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
         /* led is occluded */
         if(m_bShowRays) {
            m_vecCheckedRays.push_back(std::make_pair(true, m_cOcclusionCheckRay));
         }
         return true;
      }
      else {
         if(m_bShowRays) {
            m_vecCheckedRays.push_back(std::make_pair(false, m_cOcclusionCheckRay));
         }
      }               
      m_tReadings.emplace_back(c_led.GetColor(), ProjectOntoSensor(cLedPosition));
      return true;
   }

   /****************************************/
   /****************************************/

   REGISTER_CAMERAS_SENSOR_ALGORITHM(CCamerasSensorLEDDetectorAlgorithm,
                                     "led_detector",
                                     "Michael Allwright [allsey87@gmail.com]",
                                     "1.0",
                                     "This algorithm detects nearby LEDs seen by the camera and\n"
                                     "returns the X and Y coordinates on the sensor",
                                     "This algorithm detects nearby LEDs seen by the camera and\n"
                                     "returns the X and Y coordinates on the sensor",
                                     "Under development");  
}
