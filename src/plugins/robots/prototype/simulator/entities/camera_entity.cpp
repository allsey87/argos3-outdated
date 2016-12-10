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
      m_cFieldOfView(CRadians::ZERO),
      m_fRange(0.0f),
      m_fHorizontalResolution(0.0),
      m_fVerticalResolution(0.0) {
      SetEnabled(false);
      SetCanBeEnabledIfDisabled(true);
   }

   /****************************************/
   /****************************************/

   CCameraEntity::CCameraEntity(CComposableEntity* pc_parent,
                               const CRadians& c_field_of_view,
                               const CRadians& c_roll,
                               Real f_range,
                               Real f_horizontal_resolution,
                               Real f_vertical_resolution) :
      CEntity(pc_parent),
      m_cFieldOfView(c_field_of_view),
      m_cRoll(c_roll),
      m_fRange(f_range),
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
         bool bEnabled = false;
         GetNodeAttributeOrDefault(t_tree, "enabled", bEnabled, bEnabled);
         SetEnabled(bEnabled);
         CDegrees cFieldOfView;
         GetNodeAttribute(t_tree, "field_of_view", cFieldOfView);
         m_cFieldOfView = ToRadians(cFieldOfView);
         CDegrees cRoll;
         GetNodeAttribute(t_tree, "roll", cRoll);
         m_cRoll = ToRadians(cRoll);
         GetNodeAttribute(t_tree, "range", m_fRange);
         /* get pairs */
         Real pfTwoValues[2];
         /* resolution */
         std::string strResolution;
         GetNodeAttribute(t_tree, "resolution", strResolution);
         ParseValues<Real>(strResolution, 2, pfTwoValues, ',');
         m_fHorizontalResolution = pfTwoValues[0];
         m_fVerticalResolution = pfTwoValues[1];
         /* init camera matrix */
         m_cCameraMatrix.SetIdentityMatrix();
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
         /* get tuples */
         Real pfThreeValues[3];
         /* init distortion parameter matrix */
         m_cDistortionParameters.SetZero();
         /* distortion parameters */
         std::string strDistortionParameters;
         GetNodeAttribute(t_tree, "distortion_parameters", strDistortionParameters);
         ParseValues<Real>(strDistortionParameters, 3, pfThreeValues, ',');
         m_cDistortionParameters(0,0) = pfThreeValues[0]; // K1
         m_cDistortionParameters(0,1) = pfThreeValues[1]; // K2
         m_cDistortionParameters(0,4) = pfThreeValues[2]; // K3
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
