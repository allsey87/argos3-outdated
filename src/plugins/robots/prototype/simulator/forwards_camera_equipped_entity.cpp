/**
 * @file <argos3/plugins/robot/prototype/simulator/forwards_camera_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "forwards_camera_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CForwardsCameraEquippedEntity::CForwardsCameraEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/
   
   CForwardsCameraEquippedEntity::CForwardsCameraEquippedEntity(CComposableEntity* pc_parent,
                                                                const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}
   
   /****************************************/
   /****************************************/

   void CForwardsCameraEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         CEntity::Init(t_tree);
         TConfigurationNodeIterator itForwardsCamera("camera");
         for(itForwardsCamera = itForwardsCamera.begin(&t_tree);
             itForwardsCamera != itForwardsCamera.end();
             ++itForwardsCamera) {
            /*Initialise the forwards camera using the XML*/
            CForwardsCameraEntity* pcForwardsCamera = new CForwardsCameraEntity(this);
            pcForwardsCamera->Init(*itForwardsCamera);
            /* Add the forwards camera to this container */
            std::string strCameraBody;
            GetNodeAttribute(*itForwardsCamera, "body", strCameraBody);
            CBodyEntity& cCameraBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strCameraBody + "]");
            CVector3 cPositionOffset;
            GetNodeAttribute(*itForwardsCamera, "position", cPositionOffset);
            CQuaternion cOrientationOffset;
            GetNodeAttribute(*itForwardsCamera, "orientation", cOrientationOffset);
            
            m_vecPositionalEntities.push_back(&cCameraBody.GetPositionalEntity());
            m_vecPositionOffsets.push_back(cPositionOffset);
            m_vecOrientationOffsets.push_back(cOrientationOffset);

            m_tForwardsCameras.push_back(pcForwardsCamera);

            AddComponent(*pcForwardsCamera);

            // DEBUG - to remove!
            fprintf(stderr, 
                    "%s setup complete!\n", 
                    (m_tForwardsCameras.back()->GetContext() + m_tForwardsCameras.back()->GetId()).c_str());
            fprintf(stderr, 
                    "\tposition = [%.3f, %.3f, %.3f]\n",
                    m_vecPositionOffsets.back().GetX(),
                    m_vecPositionOffsets.back().GetY(),
                    m_vecPositionOffsets.back().GetZ());
            CRadians cX,cY,cZ; m_vecOrientationOffsets.back().ToEulerAngles(cZ, cY, cX);
            fprintf(stderr,
                    "\torientation = [%.3f, %.3f, %.3f]\n", 
                    ToDegrees(cZ).GetValue(),
                    ToDegrees(cY).GetValue(),
                    ToDegrees(cX).GetValue());
            fprintf(stderr, 
                    "\tfield of view = %.3f\n",
                    ToDegrees(m_tForwardsCameras.back()->GetFieldOfView()).GetValue());
            fprintf(stderr, 
                    "\trange = %.3f\n", 
                    m_tForwardsCameras.back()->GetRange());
            fprintf(stderr,
                    "\tresolution = %ux%u\n",
                    m_tForwardsCameras.back()->GetHorizontalResolution(),
                    m_tForwardsCameras.back()->GetVerticalResolution());
            fprintf(stderr, 
                    "\tattached to %s\n",
                    (m_vecPositionalEntities.back()->GetContext() + m_vecPositionalEntities.back()->GetId()).c_str());
            // DEBUG

         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the prototype forwards camera equipped entity \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   const CQuaternion& CForwardsCameraEquippedEntity::GetOffsetOrientation(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecOrientationOffsets.size(),
                   "CForwardsCameraEquippedEntity::GetOffsetOrientation(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecOrientationOffsets.size() = " <<
                   m_vecOrientationOffsets.size());
      return m_vecOrientationOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CVector3& CForwardsCameraEquippedEntity::GetOffsetPosition(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionOffsets.size(),
                   "CForwardsCameraEquippedEntity::GetOffsetPosition(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionOffsets.size() = " <<
                   m_vecPositionOffsets.size());
      return m_vecPositionOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CPositionalEntity& CForwardsCameraEquippedEntity::GetPositionalEntity(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionalEntities.size(),
                   "CForwardsCameraEquippedEntity::GetPositionalEntity(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionalEntities.size() = " <<
                   m_vecPositionalEntities.size());
      return *m_vecPositionalEntities[un_index];
   }

   /****************************************/
   /****************************************/
   
   CForwardsCameraEntity& CForwardsCameraEquippedEntity::GetForwardsCamera(UInt32 un_index) {
            ARGOS_ASSERT(un_index < m_tForwardsCameras.size(),
                   "CForwardsCameraEquippedEntity::GetForwardsCamera(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tForwardsCameras.size() = " <<
                   m_tForwardsCameras.size());
      return *m_tForwardsCameras[un_index];
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CForwardsCameraEquippedEntity);

   /****************************************/
   /****************************************/

}
