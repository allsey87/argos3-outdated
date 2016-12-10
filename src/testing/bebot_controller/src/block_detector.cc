
#include "block_detector.h"

#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/angles.h>

/* OpenCV libraries */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "tag.h"
#include "block.h"

namespace argos {

   /****************************************/
   /****************************************/

   void CBlockDetector::Detect(const CCI_CamerasSensorTagDetectorAlgorithm::SReading::TList& t_tag_list,
                               const CCI_CamerasSensorLEDDetectorAlgorithm::SReading::TList& t_led_list,
                               SBlock::TList& t_block_list) {
      /* Create a list for the tag detections */
      SBlock::TList t_detections_list;
      /* TODO remove ugly hacks with const_cast */
      cv::Mat cCameraMatrix(3,3,CV_64F, const_cast<double *>(&m_cCameraMatrix(0,0)));
      cv::Mat cDistortionParameters(1,5,CV_64F, const_cast<double *>(&m_cDistortionParameters(0,0)));

      for(const CCI_CamerasSensorTagDetectorAlgorithm::SReading& s_reading : t_tag_list) {
         /* Create a block for the detection */
         t_detections_list.emplace_back();
         SBlock& sBlock = t_detections_list.back();
         /* Add an empty tag to the block and make a reference to it */
         std::vector<STag>& vecBlockTags = sBlock.Tags;
         vecBlockTags.emplace_back();
         STag& sTag = vecBlockTags.back();
         /* Copy the corners of the tags into an STag for later use */
         /* TODO change STag to use CVector2 for coordinates instead of std::pair */
         /* std::vector<CVector2>::assign() */
         sTag.Corners = {
            std::make_pair(s_reading.Corners[0].GetX(), s_reading.Corners[0].GetY()),
            std::make_pair(s_reading.Corners[1].GetX(), s_reading.Corners[1].GetY()),
            std::make_pair(s_reading.Corners[2].GetX(), s_reading.Corners[2].GetY()),
            std::make_pair(s_reading.Corners[3].GetX(), s_reading.Corners[3].GetY()),
         };
         /* Copy the tag center coordinate */
         sTag.Center = std::make_pair(s_reading.Center.GetX(), s_reading.Center.GetY());
         /* Convert points to OpenCV format */
         std::vector<cv::Point2d> vecImagePts = {
            cv::Point2d(s_reading.Corners[0].GetX(), s_reading.Corners[0].GetY()),
            cv::Point2d(s_reading.Corners[1].GetX(), s_reading.Corners[1].GetY()),
            cv::Point2d(s_reading.Corners[2].GetX(), s_reading.Corners[2].GetY()),
            cv::Point2d(s_reading.Corners[3].GetX(), s_reading.Corners[3].GetY()),
         };
         cv::solvePnP(m_vecTagPts,
                      vecImagePts,
                      cCameraMatrix,
                      cDistortionParameters,
                      sTag.RotationVector,
                      sTag.TranslationVector);
         /* Detect the LEDs surrounding the tag */
         //DetectLeds(sTag, pt_image_y, pt_image_u, pt_image_v);
         /* Compose the tag-to-block and camera-to-tag transformations to get the camera-to-block transformation */
         cv::composeRT(m_cTagToBlockRotationCV,
                       m_cTagToBlockTranslationCV,
                       sTag.RotationVector,
                       sTag.TranslationVector,
                       sBlock.RotationVector,
                       sBlock.TranslationVector);
         /* calculate the center coordinate the the block */
         std::vector<cv::Point2d> vecBlockCentrePixel;
         cv::projectPoints(m_vecOriginPts,
                           sBlock.RotationVector,
                           sBlock.TranslationVector,
                           cCameraMatrix,
                           cDistortionParameters,
                           vecBlockCentrePixel);
         sBlock.Coordinates.Set(vecBlockCentrePixel[0].x, vecBlockCentrePixel[0].y);
         /* normalise the Z angle of the tag and convert to argos::CVector3 and argos::CQuaternion format */
         argos::CRotationMatrix3 cRotationMatrix;
         cv::Rodrigues(sBlock.RotationVector, cv::Mat(3, 3, CV_64F, &cRotationMatrix(0,0)));
         argos::CQuaternion cRotation = cRotationMatrix.ToQuaternion();
         argos::CRadians cBlockEulerAngles[3];
         cRotation.ToEulerAngles(cBlockEulerAngles[0], cBlockEulerAngles[1], cBlockEulerAngles[2]);
         /* TODO: this is a hack  - use proper logic here involving transforms to get to the correct orientation */
         if(cBlockEulerAngles[2].GetValue() > 0.0f) {
            argos::CQuaternion cRotX90;
            cRotX90.FromEulerAngles(argos::CRadians::ZERO, argos::CRadians::ZERO, -argos::CRadians::PI_OVER_TWO);
            cRotX90 *= cRotation;
            /* for reasons involving magic and fairy dust, switching the Z and Y angle and negating some stuff just works */
            cRotX90.ToEulerAngles(cBlockEulerAngles[1], cBlockEulerAngles[0], cBlockEulerAngles[2]);
            cBlockEulerAngles[1] = -cBlockEulerAngles[1];
         }
         /* wrap the angle into the correct range */
         for(argos::CRadians& c_angle : cBlockEulerAngles) {
            m_cBlockRotationRange.WrapValue(c_angle);
         }
         sBlock.Rotation.FromEulerAngles(cBlockEulerAngles[0], cBlockEulerAngles[1], cBlockEulerAngles[2]);
         sBlock.Translation.Set(sBlock.TranslationVector(0), sBlock.TranslationVector(1), sBlock.TranslationVector(2));

         //std::cout << "R = [" << sBlock.Rotation << "]" << std::endl;
         std::cout << "T = [" << sBlock.Translation << "]" << std::endl;
      }

      /* cluster the blocks */
      ClusterDetections(t_detections_list, t_block_list);
   }

   /****************************************/
   /****************************************/

   void CBlockDetector::ClusterDetections(SBlock::TList& t_detections_list,
                                        SBlock::TList& t_block_list) {
      
      /* some typedefs to avoid going insane */                                  
      using TCluster = SBlock::TList;
      using TClusterList = std::list<TCluster>;
      /* a working list of clusters */
      TClusterList lstClusters;
      /* loop until we have allocated all of our detections into clusters */
      while(!t_detections_list.empty()) {
         /* take a reference to the first block in the detections list */
         SBlock::TList::iterator itDetectedBlock = std::begin(t_detections_list);
         /* keep a list of interators into the matching clusters */
         std::list<TClusterList::iterator> lstBlockToClusterAssignments;
         /* for each cluster */
         for(TClusterList::iterator it_cluster = std::begin(lstClusters);
             it_cluster != std::end(lstClusters);
             it_cluster++) {
            /* for each block in the cluster */
            for(TCluster::iterator it_block = it_cluster->begin();
                it_block != it_cluster->end();
                it_block++) {
               double fInterblockDist = argos::Distance(it_block->Translation, itDetectedBlock->Translation);
               /* if the given block in this cluster is within a distance of 
                  (m_fBlockSideLength / 2) of the detected block, they belong 
                  to the same cluster */
               if(fInterblockDist < (m_fBlockSideLength / 2)) {
                  lstBlockToClusterAssignments.push_back(it_cluster);
                  /* at this point we know that this cluster is a 
                     candidate for the block and we can stop */
                  break;
               }
            }
         }
         /* At this point we have searched all the clusters */
         if(lstBlockToClusterAssignments.size() == 0) {
            /* no matches - create a new cluster in the cluster list */
            lstClusters.emplace_back();
            /* take a reference to the newly created cluster */
            TCluster& tCluster = lstClusters.back();
            /* move our detected block into the cluster */
            tCluster.splice(std::begin(tCluster),
                            t_detections_list,
                            itDetectedBlock);
         }
         else {        
            /* move the detected block into the first matching clusters */
            TClusterList::iterator itCluster = lstBlockToClusterAssignments.front();
            /* add the detected block into the first matching cluster */
            itCluster->splice(std::begin(*itCluster),
                              t_detections_list,
                              itDetectedBlock);
            /* if there was more than one matching cluster, merge them */
            if(lstBlockToClusterAssignments.size() > 1) {
               /* take an iterator to the first (destination) cluster */
               std::list<TClusterList::iterator>::iterator itDestinationCluster =
                  std::begin(lstBlockToClusterAssignments);
               /* move the blocks from the other (source) clusters into the destination cluster */
               for(std::list<TClusterList::iterator>::iterator itSourceCluster = std::next(itDestinationCluster);
                   itSourceCluster != std::end(lstBlockToClusterAssignments);
                   itSourceCluster++) {
                  /* move all the blocks from the source cluster to the destination cluster */
                  (*itDestinationCluster)->splice(std::begin(**itDestinationCluster), **itSourceCluster);
                  /* remove the empty source cluster */
                  lstClusters.erase(*itSourceCluster);
               }
            }
         }
      }

      /* find the average location of the block */
      for(TCluster& t_cluster : lstClusters) {
         argos::CVector3 cAvgLocation;
         for(SBlock& s_block : t_cluster) {
            cAvgLocation += s_block.Translation;
         }
         cAvgLocation /= t_cluster.size();
         
         /* Find the block that has the highest tag (lowest Y coordinate in 2D) */
         TCluster::iterator itTopTagBlock = std::min_element(std::begin(t_cluster),
                                            std::end(t_cluster),
                                            [] (const SBlock& s_block_a, 
                                                const SBlock& s_block_b) {
            return (s_block_a.Tags[0].Center.second 
                  < s_block_b.Tags[0].Center.second);
         });
               
         /* update it's location with the average location */
         /* perhaps it is better to turn this off - top tag will likely be the best measurement */
         //itTopTagBlock->Translation = cAvgLocation;
         
         /* This is a hack to conserve the other detected tags */
         for(TCluster::iterator it_block = std::begin(t_cluster);
            it_block != std::end(t_cluster);
            it_block++) {
            if(itTopTagBlock != it_block) {
               itTopTagBlock->HackTags.push_back(it_block->Tags[0]);
            }
         }
         
         /* Move itTopTagBlock into our list of blocks */
         /* TODO: move the other tags into the block structure */
         t_block_list.splice(std::begin(t_block_list), t_cluster, itTopTagBlock);
      }
      
   }

   /****************************************/
   /****************************************/
}

