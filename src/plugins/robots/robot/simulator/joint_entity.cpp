/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "joint_entity.h"

#include <argos3/plugins/robots/robot/simulator/frame_equipped_entity.h>

namespace argos {

   /****************************************/
   /****************************************/

   CJointEntity::CJointEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent),
      m_bDisableCollisions(false),
      m_pcFrameEquippedEntity(NULL),
      m_cLinearLowerLimit(0.0f,0.0f,0.0f),
      m_cLinearUpperLimit(0.0f,0.0f,0.0f),
      m_cAngularLowerLimit(0.0f,0.0f,0.0f),
      m_cAngularUpperLimit(0.0f,0.0f,0.0f) {}

   /****************************************/
   /****************************************/

   
   CJointEntity::CJointEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              bool b_disable_collisions) :
      CComposableEntity(pc_parent, str_id),
      m_bDisableCollisions(b_disable_collisions),
      m_cLinearLowerLimit(0.0f,0.0f,0.0f),
      m_cLinearUpperLimit(0.0f,0.0f,0.0f),
      m_cAngularLowerLimit(0.0f,0.0f,0.0f),
      m_cAngularUpperLimit(0.0f,0.0f,0.0f) {
 
      m_pcFrameEquippedEntity = new CFrameEquippedEntity(this, "frames");
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);

         /* check if we are disabling collisions */
         GetNodeAttributeOrDefault(t_tree, "disable_collisions", m_bDisableCollisions, false);

         m_pcFrameEquippedEntity = new CFrameEquippedEntity(this, "frames");
         if(NodeExists(t_tree, "frames")) {
            m_pcFrameEquippedEntity->Init(GetNode(t_tree, "frames"));
         }
         else {
            //todo what happens here, error here or later in dynamics engine?
         }

         AddComponent(*m_pcFrameEquippedEntity);
         /* parse the joints limits if they exist */
         if(NodeExists(t_tree, "limits")) {
            TConfigurationNode& tLimits = GetNode(t_tree, "limits");
            if(NodeExists(tLimits, "linear")) {
               TConfigurationNode& tLinearLimits = GetNode(tLimits, "linear");
               if(NodeAttributeExists(tLinearLimits, "lowerbound") &&
                  NodeAttributeExists(tLinearLimits, "upperbound")) {
                  GetNodeAttribute(tLinearLimits, "lowerbound", m_cLinearLowerLimit);
                  GetNodeAttribute(tLinearLimits, "upperbound", m_cLinearUpperLimit);
               }
               else {
                  THROW_ARGOSEXCEPTION("Error in parsing the joint's linear limits. You must specify both an upper and lower bound");
               }
            }
            if(NodeExists(tLimits, "angular")) {
               TConfigurationNode& tAngularLimits = GetNode(tLimits, "angular");
               if(NodeAttributeExists(tAngularLimits, "lowerbound") &&
                  NodeAttributeExists(tAngularLimits, "upperbound")) {
                  GetNodeAttribute(tAngularLimits, "lowerbound", m_cAngularLowerLimit);
                  GetNodeAttribute(tAngularLimits, "upperbound", m_cAngularUpperLimit);
               }
               else {
                  THROW_ARGOSEXCEPTION("Error in parsing the joint's angular limits. You must specify both an upper and lower bound");
               }
            }
         }
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Error while initializing joint entity", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Reset() {
      CComposableEntity::Reset();
      m_pcFrameEquippedEntity->Reset();
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Destroy() {
      m_pcFrameEquippedEntity->Destroy();      
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   void CJointEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CJointEntity);

   /****************************************/
   /****************************************/

}
