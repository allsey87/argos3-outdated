/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_forwards_camera_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "prototype_forwards_camera_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CPrototypeForwardsCameraEquippedEntity::CPrototypeForwardsCameraEquippedEntity(CComposableEntity* pc_parent) :
      CEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/
   /*
   CPrototypeForwardsCameraEquippedEntity::CPrototypeForwardsCameraEquippedEntity(CComposableEntity* pc_parent,
   AddForwardsCamera etc //TODO
   */
   /****************************************/
   /****************************************/

   void CPrototypeForwardsCameraEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         TConfigurationNode& tCamNode = GetNode(t_tree, "camera");
         CEntity::Init(tCamNode);
         GetNodeAttribute(tCamNode, "position", m_cOffsetPosition);
         GetNodeAttribute(tCamNode, "orientation", m_cOffsetOrientation);
         CDegrees cFieldOfView;
         GetNodeAttribute(tCamNode, "field_of_view", cFieldOfView);
         m_cFieldOfView = ToRadians(cFieldOfView);
         GetNodeAttribute(tCamNode, "range", m_fRange);
         GetNodeAttribute(tCamNode, "sensor_resolution", m_cSensorResolution);

         std::string strSensorBody;
         GetNodeAttribute(tCamNode, "body", strSensorBody);
         CBodyEntity& cSensorBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strSensorBody + "]");
         m_pcPositionalEntity = &cSensorBody.GetPositionalEntity();

         fprintf(stderr, "camera setup complete!\n");
         fprintf(stderr, 
                 "\tposition = [%.3f, %.3f, %.3f]\n",
                 m_cOffsetPosition.GetX(),
                 m_cOffsetPosition.GetY(),
                 m_cOffsetPosition.GetZ());
         CRadians cX,cY,cZ; m_cOffsetOrientation.ToEulerAngles(cZ, cY, cX);
         fprintf(stderr,
                 "\torientation = [%.3f, %.3f, %.3f]\n", 
                 ToDegrees(cZ).GetValue(),
                 ToDegrees(cY).GetValue(),
                 ToDegrees(cX).GetValue());
         fprintf(stderr, "\tfield of view = %.3f\n", ToDegrees(m_cFieldOfView).GetValue());
         fprintf(stderr, "\trange = %.3f\n", m_fRange);
         fprintf(stderr,
                 "\tsensor resolution = %.1f x %.1f\n",
                 m_cSensorResolution.GetX(),
                 m_cSensorResolution.GetY());
         fprintf(stderr, "\tattached to %s\b\n", m_pcPositionalEntity->GetContext().c_str());
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the prototype forwards camera equipped entity \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CPrototypeForwardsCameraEquippedEntity);

   /****************************************/
   /****************************************/

}
