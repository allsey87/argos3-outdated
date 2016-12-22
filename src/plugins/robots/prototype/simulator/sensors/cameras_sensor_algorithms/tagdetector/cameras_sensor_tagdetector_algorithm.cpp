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
      m_pcControllableEntity(NULL),
      m_bShowRays(false),
      m_fDistanceNoiseStdDev(0.0f),
      m_pcRNG(NULL),
      m_pcTagIndex(NULL) {}

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
      /* All occlusion rays start from the camera position */
      m_cOcclusionCheckRay.SetStart(m_psData->CameraLocation);
      /* Clear the old readings */ 
      m_tReadings.clear();
      m_vecCheckedRays.clear();
      m_pcTagIndex->ForEntitiesInBoxRange(m_psData->BoundingBoxPosition,
                                          m_psData->BoundingBoxHalfExtents,
                                          *this);
   }

   /****************************************/
   /****************************************/

   bool CCamerasSensorTagDetectorAlgorithm::operator()(CTagEntity& c_tag) {
      /* observable angle check (only done against tag center) */
      if(GetAngleWithCamera(c_tag) > c_tag.GetObservableAngle()) {
         return true;
      }
      /* compute the corner locations */
      const CVector3& cTagPosition = c_tag.GetPosition();
      std::vector<CVector3> vecTagCorners;
      vecTagCorners.reserve(m_vecTagCornerOffsets.size());
      for(const CVector3& c_corner_offset : m_vecTagCornerOffsets) {
         CVector3 cCorner(c_corner_offset * c_tag.GetSideLength());
         cCorner.Rotate(c_tag.GetOrientation());
         cCorner += cTagPosition;
         vecTagCorners.push_back(cCorner);
      }
      /* frustum checking */
      for(const CVector3& c_corner : vecTagCorners) {
         if(IsPointInsideFrustum(c_corner) == false) {
            /* corner is not inside the frustum */
            return true;
         }
      }
      /* ray checking */
      for(const CVector3& c_corner : vecTagCorners) {
         m_cOcclusionCheckRay.SetEnd(c_corner);
         if(GetClosestEmbodiedEntityIntersectedByRay(m_sIntersectionItem, m_cOcclusionCheckRay)) {
            /* corner is occluded */
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
      }
      /* project points and store readings */
      std::vector<CVector2> vecCornerPixels;
      vecCornerPixels.reserve(vecTagCorners.size());
      for(const CVector3& c_corner : vecTagCorners) {
         vecCornerPixels.emplace_back(ProjectOntoSensor(c_corner));
      }
      m_tReadings.emplace_back(c_tag.GetPayload(), ProjectOntoSensor(cTagPosition), vecCornerPixels);
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
