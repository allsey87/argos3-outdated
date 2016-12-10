#ifndef BLOCK_SENSOR_H
#define BLOCK_SENSOR_H

namespace argos {
   // forward declarations
}
//#include <opencv2/core/core.hpp>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/range.h>

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_tagdetector_algorithm.h>

#include <argos3/core/utility/math/matrix/squarematrix.h>
#include <argos3/core/utility/math/matrix/matrix.h>

#include "block.h"

#include <list>


namespace argos {

   class CBlockDetector {
      
   public:
      /* constructor */
      CBlockDetector() {}
      
      /* destructor */
      ~CBlockDetector() {}

      void Detect(const CCI_CamerasSensorTagDetectorAlgorithm::SReading::TList& t_tag_list,
                  const CCI_CamerasSensorLEDDetectorAlgorithm::SReading::TList& t_led_list,
                  SBlock::TList& t_block_list);

      void SetCameraMatrix(const CSquareMatrix<3>& c_camera_matrix) {
         m_cCameraMatrix = c_camera_matrix;
      }

      void SetDistortionParameters(const CMatrix<1,5>& c_parameters) {
         m_cDistortionParameters = c_parameters;
      }

   private:
      //void DetectLeds(STag& s_tag, image_u8_t* pt_y_frame, image_u8_t* pt_u_frame, image_u8_t* pt_v_frame);

      void ClusterDetections(SBlock::TList& t_detection_list,
                             SBlock::TList& t_block_list);

      /* Apriltag (w.r.t. black frame) and block side length in meters */
      const double m_fTagSize = 0.024;
      const double m_fBlockSideLength = 0.055;
      const double m_fInterLedLength = 0.040;
      const unsigned int m_unLedRegionOfInterestLength = 9;
      const unsigned int m_unLedLuminanceOnThreshold = 64;

      const std::vector<cv::Point3d> m_vecTagPts = {
         cv::Point3d(-m_fTagSize * 0.5f, -m_fTagSize * 0.5f, 0),
         cv::Point3d( m_fTagSize * 0.5f, -m_fTagSize * 0.5f, 0),
         cv::Point3d( m_fTagSize * 0.5f,  m_fTagSize * 0.5f, 0),
         cv::Point3d(-m_fTagSize * 0.5f,  m_fTagSize * 0.5f, 0),
      };

      const std::vector<cv::Point3d> m_vecOriginPts = {
         cv::Point3d(0.0f,0.0f, 0.0f)
      };

      /* Tag to block translation and rotation constants */
      const cv::Matx31d m_cTagToBlockTranslationCV = cv::Matx31d(0, 0, m_fBlockSideLength / 2);
      const cv::Matx31d m_cTagToBlockRotationCV = cv::Matx31d(0, 0, 0);

      const argos::CRange<argos::CRadians> m_cBlockRotationRange = 
         argos::CRange<argos::CRadians>(-argos::CRadians::PI_OVER_FOUR, argos::CRadians::PI_OVER_FOUR);

      /* camera matrix */
      CSquareMatrix<3> m_cCameraMatrix;
      /* camera distortion parameters */
      CMatrix<1,5> m_cDistortionParameters;  
   };
}

#endif


