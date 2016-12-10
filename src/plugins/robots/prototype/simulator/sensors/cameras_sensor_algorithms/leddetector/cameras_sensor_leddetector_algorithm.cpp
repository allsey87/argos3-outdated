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
      m_pcCameraEquippedEntity(NULL),
      m_pcControllableEntity(NULL),
      m_unCameraIndex(0),
      m_bShowRays(false),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_pcLEDIndex(NULL),
      m_unHorizontalResolution(0u),
      m_unVerticalResolution(0u) {}

   /****************************************/
   /****************************************/   

   void CCamerasSensorLEDDetectorAlgorithm::SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index) {
      m_pcCameraEquippedEntity = &c_entity;
      m_unCameraIndex = un_index;
      m_unHorizontalResolution   = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetHorizontalResolution();
      m_unVerticalResolution     = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetVerticalResolution();
      m_cCameraMatrix            = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetCameraMatrix();
      /* get the body camera to body transform */
      m_cCameraToBodyTransform.SetFromComponents(m_pcCameraEquippedEntity->GetOffsetOrientation(m_unCameraIndex),
                                                 m_pcCameraEquippedEntity->GetOffsetPosition(m_unCameraIndex));
   }

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
      CTransformationMatrix3 cBodyToWorldTransform(m_pcCameraEquippedEntity->GetPositionalEntity(m_unCameraIndex).GetOrientation(),
                                                   m_pcCameraEquippedEntity->GetPositionalEntity(m_unCameraIndex).GetPosition());
      /* build the camera to world matrix */
      (cBodyToWorldTransform * m_cCameraToBodyTransform).GetInverse().GetSubMatrix(m_cCameraToWorldMatrix,0,0);
      /* All occlusion rays start from the camera position */
      m_cOcclusionCheckRay.SetStart(m_sViewport.CameraLocation);
      /* Clear the old readings */ 
      m_tReadings.clear();
      m_vecCheckedRays.clear();
      m_pcLEDIndex->ForEntitiesInBoxRange(m_sViewport.Position,
                                          m_sViewport.HalfExtents,
                                          *this);
   }

   /****************************************/
   /****************************************/

   bool CCamerasSensorLEDDetectorAlgorithm::operator()(CLEDEntity& c_led) {
      if(c_led.GetColor() != CColor::BLACK) {
         const CVector3& cLedPosition = c_led.GetPosition();
         if((cLedPosition - m_sViewport.Position).Length() < m_sViewport.HalfExtents[0]) {
            m_cOcclusionCheckRay.SetEnd(cLedPosition);
            if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
               m_tReadings.push_back(SReading(c_led.GetColor(), Project(cLedPosition)));
               if(m_bShowRays) {
                  m_vecCheckedRays.push_back(std::pair<bool, CRay3>(false, CRay3(m_sViewport.CameraLocation, c_led.GetPosition())));
               }
            }
         }
      }
      return true;
   }

   /****************************************/
   /****************************************/

   CVector2 CCamerasSensorLEDDetectorAlgorithm::Project(const CVector3& c_vector) {
      CMatrix<4,1> cPosVector {c_vector.GetX(), c_vector.GetY(), c_vector.GetZ(), 1};
      CMatrix<3,1> cPosCamCoords(m_cCameraToWorldMatrix * cPosVector);
      /* normalize */
      cPosCamCoords(0,0) /= cPosCamCoords(2,0);
      cPosCamCoords(1,0) /= cPosCamCoords(2,0);
      cPosCamCoords(2,0) /= cPosCamCoords(2,0);
      /* get image coordinates */              
      CMatrix<3,1> cPosImgCoords(m_cCameraMatrix * cPosCamCoords);
      /* return as vector2 */
      return CVector2(cPosImgCoords(0,0), cPosImgCoords(1,0));
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
