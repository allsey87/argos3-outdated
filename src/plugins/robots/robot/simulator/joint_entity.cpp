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
      m_pcFrameEquippedEntity(NULL) {}

   /****************************************/
   /****************************************/

   
   CJointEntity::CJointEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              bool b_disable_collisions) :
      CComposableEntity(pc_parent, str_id),
      m_bDisableCollisions(b_disable_collisions) {
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

         m_pcFrameEquippedEntity = new CFrameEquippedEntity(this);
         if(NodeExists(t_tree, "frames")) {
            m_pcFrameEquippedEntity->Init(GetNode(t_tree, "frames"));
         }
         else {
            //todo what happens here, error here or later in dynamics engine?
         }
         AddComponent(*m_pcFrameEquippedEntity);


         /* parse the degrees of freedom if they exist */
         if(NodeExists(t_tree, "dof")) {
            TConfigurationNode& tDof = GetNode(t_tree, "dof");
            if(NodeExists(tDof, "linear")) {
               TConfigurationNode& tLinearDofs = GetNode(tDof, "linear");
               /* Check for degrees of freedom for linear X */
               if(NodeAttributeExists(tLinearDofs, "x" )) {
                  std::string strDofs;
                  GetNodeAttribute(tLinearDofs, "x", strDofs);
                  if(strDofs == "free") {
                     m_sLinearDofs.m_sX.m_bUnconstrained = true;
                  }
                  else {
                     GetNodeAttribute(tLinearDofs, "x", m_sLinearDofs.m_sX.m_cLimits);
                  }
               }
               /* Check for degrees of freedom for linear Y */
               if(NodeAttributeExists(tLinearDofs, "y" )) {
                  std::string strDofs;
                  GetNodeAttribute(tLinearDofs, "y", strDofs);
                  if(strDofs == "free") {
                     m_sLinearDofs.m_sY.m_bUnconstrained = true;
                  }
                  else {
                     GetNodeAttribute(tLinearDofs, "y", m_sLinearDofs.m_sY.m_cLimits);
                  }
               }
               /* Check for degrees of freedom for linear X */
               if(NodeAttributeExists(tLinearDofs, "z" )) {
                  std::string strDofs;
                  GetNodeAttribute(tLinearDofs, "z", strDofs);
                  if(strDofs == "free") {
                     m_sLinearDofs.m_sZ.m_bUnconstrained = true;
                  }
                  else {
                     GetNodeAttribute(tLinearDofs, "z", m_sLinearDofs.m_sZ.m_cLimits);
                  }
               }
            }
            if(NodeExists(tDof, "angular")) {
               TConfigurationNode& tAngularDofs = GetNode(tDof, "angular");
               /* Check for degrees of freedom for angular X */
               if(NodeAttributeExists(tAngularDofs, "x" )) {
                  std::string strDofs;
                  GetNodeAttribute(tAngularDofs, "x", strDofs);
                  if(strDofs == "free") {
                     m_sAngularDofs.m_sX.m_bUnconstrained = true;
                  }
                  else {
                     CRange<CDegrees> cAngularDofRange;
                     GetNodeAttribute(tAngularDofs, "x", cAngularDofRange);
                     m_sAngularDofs.m_sX.m_cLimits.Set(ToRadians(cAngularDofRange.GetMin()),
                                                       ToRadians(cAngularDofRange.GetMax()));
                  }
               }
               /* Check for degrees of freedom for angular Y */
               if(NodeAttributeExists(tAngularDofs, "y" )) {
                  std::string strDofs;
                  GetNodeAttribute(tAngularDofs, "y", strDofs);
                  if(strDofs == "free") {
                     m_sAngularDofs.m_sY.m_bUnconstrained = true;
                  }
                  else {
                     CRange<CDegrees> cAngularDofRange;
                     GetNodeAttribute(tAngularDofs, "y", cAngularDofRange);
                     m_sAngularDofs.m_sY.m_cLimits.Set(ToRadians(cAngularDofRange.GetMin()),
                                                       ToRadians(cAngularDofRange.GetMax()));
                  }
               }
               /* Check for degrees of freedom for angular X */
               if(NodeAttributeExists(tAngularDofs, "z" )) {
                  std::string strDofs;
                  GetNodeAttribute(tAngularDofs, "z", strDofs);
                  if(strDofs == "free") {
                     m_sAngularDofs.m_sZ.m_bUnconstrained = true;
                  }
                  else {
                     CRange<CDegrees> cAngularDofRange;
                     GetNodeAttribute(tAngularDofs, "z", cAngularDofRange);
                     m_sAngularDofs.m_sZ.m_cLimits.Set(ToRadians(cAngularDofRange.GetMin()),
                                                       ToRadians(cAngularDofRange.GetMax()));
                  }
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
