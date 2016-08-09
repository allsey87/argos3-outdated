/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/tagdetector/cameras_sensor_tagdetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_sensor_tagdetector_algorithm.h"

#include <argos3/core/simulator/simulator.h>

#include <argos3/plugins/robots/prototype/simulator/entities/tag_entity.h>
#include <argos3/plugins/robots/prototype/simulator/media/tag_medium.h>

namespace argos {

   /****************************************/
   /****************************************/   

   CCamerasSensorTagDetectorAlgorithm::CCamerasSensorTagDetectorAlgorithm() :
      m_pcCameraEquippedEntity(NULL),
      m_pcControllableEntity(NULL),
      m_unCameraIndex(0),
      m_bShowRays(false),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_pcTagIndex(NULL),
      m_unHorizontalResolution(0u),
      m_unVerticalResolution(0u) {}

   /****************************************/
   /****************************************/   

   void CCamerasSensorTagDetectorAlgorithm::SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index) {
      m_pcCameraEquippedEntity = &c_entity;
      m_unCameraIndex = un_index;

      m_cCameraPositionOffset    = m_pcCameraEquippedEntity->GetOffsetPosition(m_unCameraIndex);
      m_cCameraOrientationOffset = m_pcCameraEquippedEntity->GetOffsetOrientation(m_unCameraIndex);
      m_unHorizontalResolution   = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetHorizontalResolution();
      m_unVerticalResolution     = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetVerticalResolution();
   }

   /****************************************/
   /****************************************/
   
   void CCamerasSensorTagDetectorAlgorithm::Init(TConfigurationNode& t_tree) {
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
         m_pcTagIndex = &(CSimulator::GetInstance().GetMedium<CTagMedium>(strMedium).GetIndex());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the tag detector algorithm", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CCamerasSensorTagDetectorAlgorithm::Update() {
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

      m_pcTagIndex->ForEntitiesInBoxRange(m_sViewport.Position,
                                          m_sViewport.HalfExtents,
                                          *this);
   }

   /****************************************/
   /****************************************/

   bool CCamerasSensorTagDetectorAlgorithm::operator()(CTagEntity& c_tag) {
      if((c_tag.GetPosition() - m_sViewport.Position).Length() < m_sViewport.HalfExtents[0]) {
         m_cOcclusionCheckRay.SetEnd(c_tag.GetPosition());         
         if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
            /* Take position of current tag */


            CQuaternion cXFlip; cXFlip.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::PI);          
            
            /* south [90.0, 0.0, -135.0]
               top [90.0, -0.0, 135.0] */
            //CQuaternion cTagOrientationCam = m_cCameraOrientationOffset.Inverse();
            //cTagOrientationCam *= c_tag.GetOrientation();

            /* south [90.0, 0.0, -135.0]
               top [90.0, -0.0, 135.0] */
            // No difference from above -> should be this way, attached body (for camera) orientation is currently 0,0,0
            CQuaternion cTagOrientationCam = (m_cCameraOrientationOffset * cXFlip).Inverse();
            cTagOrientationCam *= m_cAttachedBodyOrientation.Inverse();
            cTagOrientationCam *= c_tag.GetOrientation();


            /*  */
            // No difference from above -> should be this way, attached body (for camera) orientation is currently 0,0,0
            //CQuaternion cTagOrientationCam = c_tag.GetOrientation();

            //CQuaternion cTest; cTest.FromEulerAngles(CRadians::ZERO, CRadians::ZERO, CRadians::PI);          
            //cTagOrientationCam *= cTest;

            //cTagOrientationCam *= (m_cAttachedBodyOrientation * m_cCameraOrientationOffset).Inverse();

            std::cout << cTagOrientationCam.Length() << std::endl;
            //cTagOrientationCam *= m_cCameraOrientationOffset.Inverse();

            /*
            CRadians pcAttachedBodyEulers[3];
            m_cAttachedBodyOrientation.ToEulerAngles(pcAttachedBodyEulers[0], pcAttachedBodyEulers[1], pcAttachedBodyEulers[2]);
            std::cout << "camera " << std::fixed << std::setprecision(1) << " [" 
               << ToDegrees(pcAttachedBodyEulers[0]).GetValue() << ", "
               << ToDegrees(pcAttachedBodyEulers[1]).GetValue() << ", "
               << ToDegrees(pcAttachedBodyEulers[2]).GetValue() << "]"
               << std::endl;
            */
            CVector3 cTagPositionCam;


            /*
            CRadians cTagEulers[3];
            c_tag.GetOrientation().ToEulerAngles(cTagEulers[0], cTagEulers[1], cTagEulers[2]);
            std::cout << "orientation: " << c_tag.GetId() << std::endl;
            */




            /* Transform the position of tag into the local coordinate system of the camera */
            CVector3 cTagPositionOnSensor = c_tag.GetPosition();
            cTagPositionOnSensor -= (m_cAttachedBodyPosition);
            cTagPositionOnSensor.Rotate(m_cAttachedBodyOrientation.Inverse());
            cTagPositionOnSensor -= m_cCameraPositionOffset;
            cTagPositionOnSensor.Rotate(m_cCameraOrientationOffset.Inverse());
            /* Calculate the relevant index of the pixel presenting the centroid of the detected tag */
            UInt32 unTagHorizontalIndex = m_unHorizontalResolution * 
               (cTagPositionOnSensor.GetX() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
            UInt32 unTagVerticalIndex = m_unVerticalResolution *
               (cTagPositionOnSensor.GetY() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
            m_tReadings.push_back(
               SReading(c_tag.GetPayload(),
                        unTagHorizontalIndex,
                        unTagVerticalIndex,
                        c_tag.IsLocalizable() ? cTagPositionOnSensor : CVector3(),
                        c_tag.IsLocalizable() ? cTagOrientationCam : CQuaternion()));
            /* Add this tag into our readings list */
            if(m_bShowRays) {
               m_vecCheckedRays.push_back(std::pair<bool, CRay3>(false, CRay3(m_sViewport.CameraLocation, c_tag.GetPosition())));
               m_vecCheckedRays.push_back(std::pair<bool, CRay3>(true, CRay3(m_sViewport.CameraLocation, m_sViewport.Position)));
            }
         }
      }
      return true;
   }
   
   /****************************************/
   /****************************************/

   REGISTER_CAMERAS_SENSOR_ALGORITHM(CCamerasSensorTagDetectorAlgorithm,
                                     "tag_detector",
                                     "Michael Allwright [allsey87@gmail.com]",
                                     "1.0",
                                     "This algorithm detects nearby tags seen by the camera and\n"
                                     "returns the X and Y coordinates on the sensor",
                                     "This algorithm detects nearby tags seen by the camera and\n"
                                     "returns the X and Y coordinates on the sensor",
                                     "Under development");  
}
