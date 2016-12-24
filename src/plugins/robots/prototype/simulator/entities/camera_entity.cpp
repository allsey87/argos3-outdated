/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/camera_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "camera_entity.h"
#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CCameraEntity::CCameraEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent),
      m_cRange(0.0f,0.0f),
      m_fHorizontalResolution(0.0),
      m_fVerticalResolution(0.0) {
      SetEnabled(false);
      SetCanBeEnabledIfDisabled(true);
   }

   /****************************************/
   /****************************************/

   CCameraEntity::CCameraEntity(CComposableEntity* pc_parent,
                                const CRange<Real>& c_range,
                                const CSquareMatrix<3>& c_camera_matrix,
                                const CMatrix<1,5>& c_distortion_parameters,
                                Real f_horizontal_resolution,
                                Real f_vertical_resolution) :
      CEntity(pc_parent),
      m_cRange(c_range),
      m_cCameraMatrix(c_camera_matrix),
      m_cDistortionParameters(c_distortion_parameters),
      m_fHorizontalResolution(f_horizontal_resolution),
      m_fVerticalResolution(f_vertical_resolution) {
      SetEnabled(false);
      SetCanBeEnabledIfDisabled(true);
   }

   /****************************************/
   /****************************************/

   void CCameraEntity::Init(TConfigurationNode& t_tree) {
      try {
         CEntity::Init(t_tree);
         GetNodeAttribute(t_tree, "range", m_cRange);
         /* init camera matrix */
         m_cCameraMatrix.SetIdentityMatrix();
         /* get pairs and tuples */
         Real pfTwoValues[2];
         Real pfThreeValues[3];
         /* focal length */
         std::string strFocalLength;
         GetNodeAttribute(t_tree, "focal_length", strFocalLength);
         ParseValues<Real>(strFocalLength, 2, pfTwoValues, ',');
         m_cCameraMatrix(0,0) = pfTwoValues[0]; // Fx
         m_cCameraMatrix(1,1) = pfTwoValues[1]; // Fy
         /* principle point */
         std::string strPrinciplePoint;
         GetNodeAttribute(t_tree, "principle_point", strPrinciplePoint);
         ParseValues<Real>(strPrinciplePoint, 2, pfTwoValues, ',');
         m_cCameraMatrix(0,2) = pfTwoValues[0]; // Px
         m_cCameraMatrix(1,2) = pfTwoValues[1]; // Py
         /* init distortion parameter matrix */
         m_cDistortionParameters.SetZero();
         /* distortion parameters */
         std::string strDistortionParameters;
         GetNodeAttribute(t_tree, "distortion_parameters", strDistortionParameters);
         ParseValues<Real>(strDistortionParameters, 3, pfThreeValues, ',');
         m_cDistortionParameters(0,0) = pfThreeValues[0]; // K1
         m_cDistortionParameters(0,1) = pfThreeValues[1]; // K2
         m_cDistortionParameters(0,4) = pfThreeValues[2]; // K3
         /* resolution */
         std::string strResolution;
         GetNodeAttribute(t_tree, "resolution", strResolution);
         ParseValues<Real>(strResolution, 2, pfTwoValues, ',');
         m_fHorizontalResolution = pfTwoValues[0];
         m_fVerticalResolution = pfTwoValues[1];
         /* enable? */
         bool bEnabled = false;
         GetNodeAttributeOrDefault(t_tree, "enabled", bEnabled, bEnabled);
         SetEnabled(bEnabled);
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing camera entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CCameraEntity);

   /****************************************/
   /****************************************/

}
