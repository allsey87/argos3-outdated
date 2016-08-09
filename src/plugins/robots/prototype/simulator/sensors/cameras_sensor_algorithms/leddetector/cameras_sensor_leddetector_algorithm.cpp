/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/cameras_sensor_leddetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_sensor_leddetector_algorithm.h"

#include <argos3/core/simulator/simulator.h>

#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/plugins/simulator/entities/led_entity.h>

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

      m_cCameraPositionOffset    = m_pcCameraEquippedEntity->GetOffsetPosition(m_unCameraIndex);
      m_cCameraOrientationOffset = m_pcCameraEquippedEntity->GetOffsetOrientation(m_unCameraIndex);
      m_unHorizontalResolution   = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetHorizontalResolution();
      m_unVerticalResolution     = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetVerticalResolution();;
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
      // TODO: Extend CTransformationMatrix3 / CSquareMatrix<N> to compute inverse
      // This will allow faster transforms of the LED positions to the sensor
      // coordinate system

      m_cAttachedBodyPosition = m_pcCameraEquippedEntity->GetPositionalEntity(m_unCameraIndex).GetPosition();
      m_cAttachedBodyOrientation = m_pcCameraEquippedEntity->GetPositionalEntity(m_unCameraIndex).GetOrientation();

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
         if((c_led.GetPosition() - m_sViewport.Position).Length() < m_sViewport.HalfExtents[0]) {
            m_cOcclusionCheckRay.SetEnd(c_led.GetPosition());         
            if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
               /* Take position of current LED */
               CVector3 cLedPositionOnSensor = c_led.GetPosition();
               /* Transform the position of LED into the local coordinate system of the camera */
               cLedPositionOnSensor -= (m_cAttachedBodyPosition);
               cLedPositionOnSensor.Rotate(m_cAttachedBodyOrientation.Inverse());
               cLedPositionOnSensor -= m_cCameraPositionOffset;
               cLedPositionOnSensor.Rotate(m_cCameraOrientationOffset.Inverse());
               /* Calculate the relevant index of the pixel presenting the centroid of the detected LED blob */
               UInt32 unLedHorizontalIndex = m_unHorizontalResolution * (cLedPositionOnSensor.GetX() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
               UInt32 unLedVerticalIndex = m_unVerticalResolution * (cLedPositionOnSensor.GetY() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
               /* Store this led into our readings list */
               m_tReadings.push_back(SReading(c_led.GetColor(), unLedHorizontalIndex, unLedVerticalIndex));
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
