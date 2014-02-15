/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/camera_sensor_leddetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "camera_sensor_leddetector_algorithm.h"

#include <argos3/plugins/simulator/media/led_medium.h>
#include <argos3/plugins/simulator/entities/led_entity.h>


namespace argos {

   /****************************************/
   /****************************************/

   bool CCameraSensorLEDDetectorAlgorithm::operator()(CLEDEntity& c_led) {
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
               cLedPositionOnSensor.Rotate(CQuaternion(m_cCameraRoll, CVector3::Z));
               /* Calculate the relevant index of the pixel presenting the centroid of the detected LED blob */
               UInt32 unLedHorizontalIndex = m_unHorizontalResolution * (cLedPositionOnSensor.GetX() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
               UInt32 unLedVerticalIndex = m_unVerticalResolution * (cLedPositionOnSensor.GetY() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
                  /* Store this observation into our observation list */
               m_psReading->ObservationList.push_back(CCI_PrototypeCamerasSensor::SObservation(c_led.GetColor(), unLedHorizontalIndex, unLedVerticalIndex));
               if(m_bShowRays) {
                  m_cControllableEntity.AddCheckedRay(false, CRay3(m_cCameraPosition, c_led.GetPosition()));
               }
            }
            else {
               m_cControllableEntity.AddCheckedRay(true, CRay3(m_cCameraPosition, c_led.GetPosition()));
            }
         }
      }
      return true;
   }
   

   /****************************************/
   /****************************************/
   
   void CCameraSensorLEDDetectorAlgorithm::Init(TConfigurationNode& t_tree) {
      try {
         /* Parent class init */
         CCI_PrototypeCamerasSensor::Init(t_tree);
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

   void CCameraSensorLEDDetectorAlgorithm::Update() {
      m_pcLEDIndex->ForEntitiesInBoxRange(m_cViewport.Position,
                                          m_cViewport.HalfExtents,
                                          *this); 
   }

   /****************************************/
   /****************************************/
   
   REGISTER_CAMERA_SENSOR_ALGORITHM(CCameraSensorLEDDetectorAlgorithm,
                                    "led_detector",
                                    "Michael Allwright [allsey87@gmail.com]",
                                    "1.0",
                                    "This algorithm detects nearby LEDs seen by the camera and\n"
                                    "returns the X and Y coordinates on the sensor",
                                    "This algorithm detects nearby LEDs seen by the camera and\n"
                                    "returns the X and Y coordinates on the sensor",
                                    "Under development");
   
