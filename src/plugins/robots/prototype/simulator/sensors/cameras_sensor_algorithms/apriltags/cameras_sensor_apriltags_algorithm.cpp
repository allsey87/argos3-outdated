/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/cameras_sensor_apriltags_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_sensor_apriltags_algorithm.h"

#include <argos3/core/simulator/simulator.h>

#include <argos3/plugins/robots/prototype/simulator/entities/barcode2_entity.h>
#include <argos3/plugins/robots/prototype/simulator/media/barcode2_medium.h>

namespace argos {

   /****************************************/
   /****************************************/   

   CCamerasSensorApriltagsAlgorithm::CCamerasSensorApriltagsAlgorithm() :
      m_pcCameraEquippedEntity(NULL),
      m_pcControllableEntity(NULL),
      m_unCameraIndex(0),
      m_bShowRays(false),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_pcBarcodeIndex(NULL),
      m_unHorizontalResolution(0u),
      m_unVerticalResolution(0u) {}

   /****************************************/
   /****************************************/   

   void CCamerasSensorApriltagsAlgorithm::SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index) {
      m_pcCameraEquippedEntity = &c_entity;
      m_unCameraIndex = un_index;

      m_cCameraPositionOffset    = m_pcCameraEquippedEntity->GetOffsetPosition(m_unCameraIndex);
      m_cCameraOrientationOffset = m_pcCameraEquippedEntity->GetOffsetOrientation(m_unCameraIndex);
      m_cCameraRoll              = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetRoll();
      m_unHorizontalResolution   = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetHorizontalResolution();
      m_unVerticalResolution     = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetVerticalResolution();;
   }

   /****************************************/
   /****************************************/
   
   void CCamerasSensorApriltagsAlgorithm::Init(TConfigurationNode& t_tree) {
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
         m_pcBarcodeIndex = &(CSimulator::GetInstance().GetMedium<CBarcode2Medium>(strMedium).GetIndex());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the Apriltags algorithm", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CCamerasSensorApriltagsAlgorithm::Update() {
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

      m_pcBarcodeIndex->ForEntitiesInBoxRange(m_sViewport.Position,
                                              m_sViewport.HalfExtents,
                                              *this);
   }

   /****************************************/
   /****************************************/

   bool CCamerasSensorApriltagsAlgorithm::operator()(CBarcode2Entity& c_barcode) {
      if((c_barcode.GetPosition() - m_sViewport.Position).Length() < m_sViewport.HalfExtents[0]) {
         m_cOcclusionCheckRay.SetEnd(c_barcode.GetPosition());         
         if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
            /* Take position of current barcode */
            CVector3 cBarcodePositionOnSensor = c_barcode.GetPosition();
            /* Transform the position of barcode into the local coordinate system of the camera */
            cBarcodePositionOnSensor -= (m_cAttachedBodyPosition);
            cBarcodePositionOnSensor.Rotate(m_cAttachedBodyOrientation.Inverse());
            cBarcodePositionOnSensor -= m_cCameraPositionOffset;
            cBarcodePositionOnSensor.Rotate(m_cCameraOrientationOffset.Inverse());
            cBarcodePositionOnSensor.Rotate(CQuaternion(m_cCameraRoll, CVector3::Z));
            /* Calculate the relevant index of the pixel presenting the centroid of the detected barcode blob */
            UInt32 unBarcodeHorizontalIndex = m_unHorizontalResolution * (cBarcodePositionOnSensor.GetX() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
            UInt32 unBarcodeVerticalIndex = m_unVerticalResolution * (cBarcodePositionOnSensor.GetY() + m_sViewport.HalfExtents[0]) / (2.0f * m_sViewport.HalfExtents[0]);
            if(c_barcode.IsLocalizable()) {
               m_tReadings.push_back(SReading(c_barcode.GetPayload(),
                                              unBarcodeHorizontalIndex,
                                              unBarcodeVerticalIndex,
                                              CVector3(),
                                              CQuaternion()));
            }
            else {
               m_tReadings.push_back(SReading(c_barcode.GetPayload(),
                                              unBarcodeHorizontalIndex,
                                              unBarcodeVerticalIndex));
            }
            /* Store this barcode into our readings list */
            if(m_bShowRays) {
               m_vecCheckedRays.push_back(std::pair<bool, CRay3>(false, CRay3(m_sViewport.CameraLocation, c_barcode.GetPosition())));
            }
         }
      }
      return true;
   }
   
   /****************************************/
   /****************************************/

   REGISTER_CAMERAS_SENSOR_ALGORITHM(CCamerasSensorApriltagsAlgorithm,
                                     "apriltags",
                                     "Michael Allwright [allsey87@gmail.com]",
                                     "1.0",
                                     "This algorithm detects nearby barcodes seen by the camera and\n"
                                     "returns the X and Y coordinates on the sensor",
                                     "This algorithm detects nearby barcodes seen by the camera and\n"
                                     "returns the X and Y coordinates on the sensor",
                                     "Under development");  
}
