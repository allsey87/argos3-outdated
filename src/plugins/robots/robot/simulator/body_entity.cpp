/**
 * @file <argos3/plugins/robots/robot/simulator/body_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "body_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CBodyEntity::CBodyEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CBodyEntity::CBodyEntity(CComposableEntity* pc_parent,
                            const std::string& str_id,
                            const CVector3& c_offset_position,
                            const CQuaternion& c_offset_orientation,
                            const CVector3& c_size,
                            Real f_mass) :
      CComposableEntity(pc_parent, str_id),
      m_cSize(c_size),
      m_fMass(f_mass) {
      
      /* default constructors are used for the positional component of the entity as
       * the position of this body is driven by the the dynamics engine. The offset 
       * component is initialised using the passed parameters */
      m_pcPositionalEntity = new CPositionalEntity(this, "absolute", CVector3(), CQuaternion());
      m_pcOffsetPositionalEntity = new CPositionalEntity(this, "relative", c_offset_position, c_offset_orientation);

      AddComponent(*m_pcPositionalEntity);
      AddComponent(*m_pcOffsetPositionalEntity);
   }

   /****************************************/
   /****************************************/

   //@todo Is this method to be provided? remove/implement
   void CBodyEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);

         /* just use the default initialisation method for the positional entity
          * as the position will be driven directly from a physics engine */
         m_pcPositionalEntity = new CPositionalEntity(this);
         AddComponent(*m_pcPositionalEntity);
         if(NodeExists(t_tree, "coordinates")) {
            m_pcPositionalEntity->Init(GetNode(t_tree, "coordinates"));
         }

         /* Parse body attributes */  
         GetNodeAttribute(t_tree, "size", m_cSize);
         GetNodeAttribute(t_tree, "mass", m_fMass);

         m_pcOffsetPositionalEntity = new CPositionalEntity(this);
         AddComponent(*m_pcOffsetPositionalEntity);
         if(NodeExists(t_tree, "offset")) {
            m_pcOffsetPositionalEntity->Init(GetNode(t_tree, "offset"));
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing body entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CBodyEntity::Reset() {
      CComposableEntity::Reset();
   }

   /****************************************/
   /****************************************/

   void CBodyEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CBodyEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CBodyEntity);

   /****************************************/
   /****************************************/

}
