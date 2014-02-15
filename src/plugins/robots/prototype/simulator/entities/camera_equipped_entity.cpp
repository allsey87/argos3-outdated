/**
 * @file <argos3/plugins/robot/prototype/simulator/entities/camera_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "camera_equipped_entity.h"
#include <argos3/core/simulator/space/space.h>
#include <argos3/plugins/robots/prototype/simulator/entities/body_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CCameraEquippedEntity::CCameraEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {
   }

   /****************************************/
   /****************************************/
   
   CCameraEquippedEntity::CCameraEquippedEntity(CComposableEntity* pc_parent,
                                                                const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}
   
   /****************************************/
   /****************************************/

   void CCameraEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         CEntity::Init(t_tree);
         TConfigurationNodeIterator itCamera("camera");
         for(itCamera = itCamera.begin(&t_tree);
             itCamera != itCamera.end();
             ++itCamera) {
            /*Initialise the  camera using the XML*/
            CCameraEntity* pcCamera = new CCameraEntity(this);
            pcCamera->Init(*itCamera);
            /* Add the  camera to this container */
            std::string strCameraBody;
            GetNodeAttribute(*itCamera, "body", strCameraBody);
            CBodyEntity& cCameraBody = GetParent().GetComponent<CBodyEntity>("bodies.body[" + strCameraBody + "]");
            CVector3 cPositionOffset;
            GetNodeAttribute(*itCamera, "position", cPositionOffset);
            CQuaternion cOrientationOffset;
            GetNodeAttribute(*itCamera, "orientation", cOrientationOffset);
            
            m_vecPositionalEntities.push_back(&cCameraBody.GetPositionalEntity());
            m_vecPositionOffsets.push_back(cPositionOffset);
            m_vecOrientationOffsets.push_back(cOrientationOffset);

            m_tCameras.push_back(pcCamera);

            AddComponent(*pcCamera);

            // // DEBUG - to remove!
            // fprintf(stderr, 
            //         "%s setup complete!\n", 
            //         (m_tCameras.back()->GetContext() + m_tCameras.back()->GetId()).c_str());
            // fprintf(stderr, 
            //         "\tposition = [%.3f, %.3f, %.3f]\n",
            //         m_vecPositionOffsets.back().GetX(),
            //         m_vecPositionOffsets.back().GetY(),
            //         m_vecPositionOffsets.back().GetZ());
            // CRadians cX,cY,cZ; m_vecOrientationOffsets.back().ToEulerAngles(cZ, cY, cX);
            // fprintf(stderr,
            //         "\torientation = [%.3f, %.3f, %.3f]\n", 
            //         ToDegrees(cZ).GetValue(),
            //         ToDegrees(cY).GetValue(),
            //         ToDegrees(cX).GetValue());
            // fprintf(stderr, 
            //         "\tfield of view = %.3f\n",
            //         ToDegrees(m_tCameras.back()->GetFieldOfView()).GetValue());
            // fprintf(stderr, 
            //         "\trange = %.3f\n", 
            //         m_tCameras.back()->GetRange());
            // fprintf(stderr,
            //         "\tresolution = %ux%u\n",
            //         m_tCameras.back()->GetHorizontalResolution(),
            //         m_tCameras.back()->GetVerticalResolution());
            // fprintf(stderr, 
            //         "\tattached to %s\n",
            //         (m_vecPositionalEntities.back()->GetContext() + m_vecPositionalEntities.back()->GetId()).c_str());
            // // DEBUG

         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error initializing the prototype camera equipped entity \"" << GetId() << "\"", ex);
      }
   }

   /****************************************/
   /****************************************/

   const CQuaternion& CCameraEquippedEntity::GetOffsetOrientation(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecOrientationOffsets.size(),
                   "CCameraEquippedEntity::GetOffsetOrientation(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecOrientationOffsets.size() = " <<
                   m_vecOrientationOffsets.size());
      return m_vecOrientationOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CVector3& CCameraEquippedEntity::GetOffsetPosition(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionOffsets.size(),
                   "CCameraEquippedEntity::GetOffsetPosition(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionOffsets.size() = " <<
                   m_vecPositionOffsets.size());
      return m_vecPositionOffsets[un_index];
   }

   /****************************************/
   /****************************************/
   
   const CPositionalEntity& CCameraEquippedEntity::GetPositionalEntity(UInt32 un_index) const {
      ARGOS_ASSERT(un_index < m_vecPositionalEntities.size(),
                   "CCameraEquippedEntity::GetPositionalEntity(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_vecPositionalEntities.size() = " <<
                   m_vecPositionalEntities.size());
      return *m_vecPositionalEntities[un_index];
   }

   /****************************************/
   /****************************************/
   
   CCameraEntity& CCameraEquippedEntity::GetCamera(UInt32 un_index) {
            ARGOS_ASSERT(un_index < m_tCameras.size(),
                   "CCameraEquippedEntity::GetCamera(), id=\"" <<
                   GetContext() + GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tCameras.size() = " <<
                   m_tCameras.size());
      return *m_tCameras[un_index];
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CCameraEquippedEntity);

   /****************************************/
   /****************************************/

}
