/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/cameras_sensor_leddetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
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
    
      /* build the homography matrix */
      CMatrix<3,4> cCameraToWorldMatrix;
      (cBodyToWorldTransform * m_cCameraToBodyTransform).GetInverse().GetSubMatrix(cCameraToWorldMatrix,0,0);
      m_cHomographyMatrix = m_cCameraMatrix * cCameraToWorldMatrix;
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
               CMatrix<4,1> cPositionVector = {cLedPosition.GetX(), cLedPosition.GetY(), cLedPosition.GetZ(), 1};
               CMatrix<3,1> cImageCoordinates(m_cHomographyMatrix * cPositionVector);
               /* Store this led into our readings list */
               m_tReadings.push_back(SReading(c_led.GetColor(),
                                              cImageCoordinates(0,0) + m_unHorizontalResolution * 0.5,
                                              cImageCoordinates(1,0) + m_unVerticalResolution * 0.5));
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
