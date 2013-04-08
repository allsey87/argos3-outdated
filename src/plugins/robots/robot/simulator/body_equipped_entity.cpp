/**
 * @file <argos3/plugins/robots/robot/simulator/body_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "body_equipped_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CBodyEquippedEntity::CBodyEquippedEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CBodyEquippedEntity::CBodyEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id) :
      CComposableEntity(pc_parent, str_id) {}

   /****************************************/
   /****************************************/

   void CBodyEquippedEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Go through the body entries */
         CVector3 cPosition;
         CVector3 cSize;
         CQuaternion cOrientation;
         Real fMass;
         
         TConfigurationNodeIterator itBody("body");
         for(itBody = itBody.begin(&t_tree);
             itBody != itBody.end();
             ++itBody) {
            
            GetNodeAttribute(*itBody, "size", cSize);
            GetNodeAttribute(*itBody, "mass", fMass);

            TConfigurationNode tOffset = GetNode(*itBody,"offset");

            GetNodeAttribute(tOffset, "position", cPosition);
            GetNodeAttribute(tOffset, "orientation", cOrientation);
            

            AddBody(cPosition, cOrientation, cSize, fMass);
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize body equipped entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBodyEquippedEntity::Reset() {
      for(CBodyEntity::TList::iterator it = m_tBodies.begin();
          it != m_tBodies.end();
          ++it) {
         (*it)->Reset();
      }
   }

   /****************************************/
   /****************************************/

   void CBodyEquippedEntity::AddBody(const CVector3& c_offset_position,
                                     const CQuaternion& c_offset_orientation,
                                     const CVector3& c_size,
                                     Real f_mass) {
      CBodyEntity* pcBody =
         new CBodyEntity(
            this,
            GetId() + ".body[" + ToString(m_tBodies.size()) + "]",
            c_offset_position,
            c_offset_orientation,
            c_size,
            f_mass);
      m_tBodies.push_back(pcBody);
      fprintf(stderr, "adding body %s\n", pcBody->GetId().c_str());
      AddComponent(*pcBody);
   }

   /****************************************/
   /****************************************/

   CBodyEntity& CBodyEquippedEntity::GetBody(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tBodies.size(),
                   "CBodyEquippedEntity::GetBody(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tBodys.size() = " <<
                   m_tBodies.size());
      return *m_tBodies[un_index];
   }

   /****************************************/
   /****************************************/

   CBodyEntity& CBodyEquippedEntity::GetBody(std::string str_id) {
      CBodyEntity::TList::iterator itBody;

      for(itBody = m_tBodies.begin();
          itBody != m_tBodies.end();
          ++itBody) {
         if((*itBody)->GetId() == str_id) {
            break;
         }   
      }
      ARGOS_ASSERT(itBody != m_tBodies.end(),
                   "CBodyEquippedEntity::GetBody(), id=\"" <<
                   GetId() <<
                   "\": body with id=\"" << str_id <<
                   "\" not found!");
      return **itBody;
   }

   void CBodyEquippedEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CBodyEquippedEntity);

   /****************************************/
   /****************************************/

}
