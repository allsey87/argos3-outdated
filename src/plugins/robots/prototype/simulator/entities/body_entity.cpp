/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/body_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "body_entity.h"

#include <argos3/plugins/robots/prototype/utility/box_geometry3.h>
#include <argos3/plugins/robots/prototype/utility/cylinder_geometry3.h>
#include <argos3/plugins/robots/prototype/utility/sphere_geometry3.h>

namespace argos {

   /****************************************/
   /****************************************/

   CBodyEntity::CBodyEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent),
      m_pcPositionalEntity(NULL),
      m_pcOffsetPositionalEntity(NULL),
      m_pcGeometry(NULL),
      m_fMass(0.0f) {}

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
         std::string strBodyGeometry;
         GetNodeAttribute(t_tree, "geometry", strBodyGeometry);
         if(strBodyGeometry == "box") {
            /* requested geometry is a box*/
            CVector3 cSize;
            GetNodeAttribute(t_tree, "size", cSize);
            m_pcGeometry = new CBoxGeometry3(cSize);
         } else if(strBodyGeometry == "cylinder") {
            /* requested geometry is a cylinder */
            Real fHeight, fRadius;
            GetNodeAttribute(t_tree, "height", fHeight);
            GetNodeAttribute(t_tree, "radius", fRadius);
            m_pcGeometry = new CCylinderGeometry3(fRadius, fHeight);
         } else if(strBodyGeometry == "sphere") {
            /* requested geometry is a sphere */
            Real fRadius;
            GetNodeAttribute(t_tree, "radius", fRadius);
            m_pcGeometry = new CSphereGeometry3(fRadius);
         } else {
            /* requested geometry is unknown */
            THROW_ARGOSEXCEPTION("Unknown geometry type " << strBodyGeometry << " provided");
         }
         GetNodeAttribute(t_tree, "mass", m_fMass);
         m_pcOffsetPositionalEntity = new CPositionalEntity(this);
         AddComponent(*m_pcOffsetPositionalEntity);
         if(NodeExists(t_tree, "offset")) {
            m_pcOffsetPositionalEntity->Init(GetNode(t_tree, "offset"));
         }
         if(NodeExists(t_tree, "attributes")) {
            TConfigurationNodeIterator itAttribute("attribute");
            for(itAttribute = itAttribute.begin(&GetNode(t_tree, "attributes"));
                itAttribute != itAttribute.end();
                ++itAttribute) {
               std::string strKey, strValue;
               GetNodeAttribute(*itAttribute, "key", strKey);
               GetNodeAttribute(*itAttribute, "value", strValue);
               m_mapAttributes.insert(std::pair<std::string, std::string>(strKey, strValue));
            }
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
      delete m_pcGeometry;
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
