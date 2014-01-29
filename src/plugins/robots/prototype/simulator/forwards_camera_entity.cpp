/**
 * @file <argos3/core/simulator/entity/forwards_camera_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "forwards_camera_entity.h"
#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CForwardsCameraEntity::CForwardsCameraEntity(CComposableEntity* pc_parent) :
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

   CForwardsCameraEntity::CForwardsCameraEntity(CComposableEntity* pc_parent,
                                                const CRadians& c_field_of_view,
                                                Real f_range,
                                                UInt32 un_horizontal_resolution,
                                                UInt32 un_vertical_resolution) :
      CEntity(pc_parent),
      m_cFieldOfView(c_field_of_view),
      m_fRange(f_range),
      m_unHorizontalResolution(un_horizontal_resolution),
      m_unVerticalResolution(un_vertical_resolution) {
      SetEnabled(false);
      SetCanBeEnabledIfDisabled(true);
   }

   /****************************************/
   /****************************************/

   void CForwardsCameraEntity::Init(TConfigurationNode& t_tree) {
      try {
         CEntity::Init(t_tree);
         bool bEnabled = false;
         GetNodeAttributeOrDefault(t_tree, "enabled", bEnabled, bEnabled);
         SetEnabled(bEnabled);
         CDegrees cRoll;
         GetNodeAttribute(t_tree, "roll", cRoll);
         m_cRoll = ToRadians(cRoll);
         CDegrees cFieldOfView;
         GetNodeAttribute(t_tree, "field_of_view", cFieldOfView);
         m_cFieldOfView = ToRadians(cFieldOfView);
         GetNodeAttribute(t_tree, "range", m_fRange);
         std::string strResolution;
         GetNodeAttribute(t_tree, "resolution", strResolution);
         UInt32 punValues[2];
         ParseValues<UInt32>(strResolution, 2, punValues, ',');
         m_unHorizontalResolution = punValues[0];
         m_unVerticalResolution = punValues[1];
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing forwards camera entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CForwardsCameraEntity);

   /****************************************/
   /****************************************/

}
