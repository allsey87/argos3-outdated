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
      m_unHorizontalResolution(0),
      m_unVerticalResolution(0) {
      SetEnabled(false);
      SetCanBeEnabledIfDisabled(true);
   }

   /****************************************/
   /****************************************/

   CCameraEntity::CCameraEntity(CComposableEntity* pc_parent,
                               const CRadians& c_field_of_view,
                               const CRadians& c_roll,
                               Real f_range,
                               UInt32 un_horizontal_resolution,
                               UInt32 un_vertical_resolution) :
      CEntity(pc_parent),
      m_cFieldOfView(c_field_of_view),
      m_cRoll(c_roll),
      m_fRange(f_range),
      m_unHorizontalResolution(un_horizontal_resolution),
      m_unVerticalResolution(un_vertical_resolution) {
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
         UInt32 punValues[2];
         /* resolution */
         std::string strResolution;
         GetNodeAttribute(t_tree, "resolution", strResolution);
         ParseValues<UInt32>(strResolution, 2, punValues, ',');
         m_unHorizontalResolution = punValues[0];
         m_unVerticalResolution = punValues[1];
         /* init camera matrix */
         m_cCameraMatrix.SetIdentityMatrix();
         /* focal length */
         std::string strFocalLength;
         GetNodeAttribute(t_tree, "focal_length", strFocalLength);
         ParseValues<UInt32>(strFocalLength, 2, punValues, ',');
         m_cCameraMatrix(0,0) = punValues[0];
         m_cCameraMatrix(1,1) = punValues[1];
         /* principle point */
         std::string strPrinciplePoint;
         GetNodeAttribute(t_tree, "principle_point", strPrinciplePoint);
         ParseValues<UInt32>(strPrinciplePoint, 2, punValues, ',');
         m_cCameraMatrix(0,2) = punValues[0];
         m_cCameraMatrix(1,2) = punValues[1];

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
