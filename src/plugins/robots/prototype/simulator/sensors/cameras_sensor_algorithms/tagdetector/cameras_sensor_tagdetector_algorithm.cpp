/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithms/tagdetector/cameras_sensor_tagdetector_algorithm.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "cameras_sensor_tagdetector_algorithm.h"

#include <argos3/core/simulator/simulator.h>

#include <argos3/plugins/robots/prototype/simulator/entities/tag_entity.h>
#include <argos3/plugins/robots/prototype/simulator/media/tag_medium.h>

#include <argos3/core/utility/math/matrix/matrix.h>
#include <argos3/core/utility/math/matrix/squarematrix.h>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>

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
      m_unHorizontalResolution   = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetHorizontalResolution();
      m_unVerticalResolution     = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetVerticalResolution();
      m_cCameraMatrix            = m_pcCameraEquippedEntity->GetCamera(m_unCameraIndex).GetCameraMatrix();
      /* get the body camera to body transform */
      m_cCameraToBodyTransform.SetFromComponents(m_pcCameraEquippedEntity->GetOffsetOrientation(m_unCameraIndex),
                                                 m_pcCameraEquippedEntity->GetOffsetPosition(m_unCameraIndex));
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
         /* Get tag medium from id specified in the XML */
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
      CTransformationMatrix3 cBodyToWorldTransform(m_pcCameraEquippedEntity->GetPositionalEntity(m_unCameraIndex).GetOrientation(),
                                                   m_pcCameraEquippedEntity->GetPositionalEntity(m_unCameraIndex).GetPosition());
      /* build the camera to world matrix */
      (cBodyToWorldTransform * m_cCameraToBodyTransform).GetInverse().GetSubMatrix(m_cCameraToWorldMatrix,0,0);
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
      const CVector3& cTagPosition = c_tag.GetPosition();
      if((cTagPosition - m_sViewport.Position).Length() < m_sViewport.HalfExtents[0]) {
         m_cOcclusionCheckRay.SetEnd(cTagPosition);
         /* note: we only ray check against the center of the tag */
         if(!GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
            /* Store this led into our readings list */
            std::vector<CVector2> vecCorners;
            for(const CVector3& cTagCornerOffset : m_vecTagCornerOffsets) {
               CVector3 cCornerPosition(cTagCornerOffset * c_tag.GetSideLength());
               cCornerPosition.Rotate(c_tag.GetOrientation());
               cCornerPosition += cTagPosition;
               vecCorners.push_back(Project(cCornerPosition));
            }
            m_tReadings.push_back(SReading(c_tag.GetPayload(),
                                           Project(cTagPosition),
                                           vecCorners));
            if(m_bShowRays) {
               m_vecCheckedRays.push_back(std::pair<bool, CRay3>(false, CRay3(m_sViewport.CameraLocation, cTagPosition)));
            }
         }
      }
      return true;
   }

   /****************************************/
   /****************************************/

   CVector2 CCamerasSensorTagDetectorAlgorithm::Project(const CVector3& c_vector) {
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
