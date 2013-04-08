/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "joint_entity.h"

namespace argos {

   /****************************************/
   /****************************************/

   CJointEntity::CJointEntity(CComposableEntity* pc_parent) :
      CComposableEntity(pc_parent) {}

   /****************************************/
   /****************************************/

   CJointEntity::CJointEntity(CComposableEntity* pc_parent,
                              const std::string& str_id,
                              EJointType e_joint_type,
                              bool b_disable_collisions,                              
                              const CBodyEntity::TList& t_connected_bodies,                              
                              const std::vector<CVector3>& vec_rotation_axes,
                              const std::vector<CVector3>& vec_rotation_points) :
                              
                              //const std::vector<Real>& vec_joint_parameters) :
      
      CComposableEntity(pc_parent, str_id),
      m_eJointType(e_joint_type),
      m_bDisableCollisions(b_disable_collisions),
      m_tConnectedBodies(t_connected_bodies),
      m_vecRotationAxes(vec_rotation_axes),
      m_vecRotationPoints(vec_rotation_points) {
      //@todo clean up the parameter passing logic / parsing etc
      //m_vecJointParameters(vec_joint_parameters) {
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);

         /* Determine the joint type */
         std::string strJointType;
         GetNodeAttribute(t_tree, "type", strJointType);

         if(strJointType == "rotary") {
            m_eJointType = ROTARY;
         }/*
         else if(strJointType == "prismatic") {
            m_eJointType = EJointType::PRIMATIC;
         }
         else if(strJointType == "pivot") {
            m_eJointType = EJointType::PIVOT;
            }*/
         else {
            THROW_ARGOSEXCEPTION("The specified joint type \"" + strJointType + "\" is not implemented.")
         }
         
         /* check if we are disabling collisions */
         GetNodeAttributeOrDefault(t_tree, "disable_collisions", m_bDisableCollisions, true);

         /* Get a reference to the bodies is the parent entity */
         //@todo make this a pass parameter?
         CBodyEquippedEntity& cBodyEquippedEntity 
            = GetParent().GetParent().GetComponent<CBodyEquippedEntity>("body_equipped_entity");

         /* Get a reference to the bodies involved in the joint relationship */
         TConfigurationNodeIterator itConnectedBody("body");
         
         for(itConnectedBody = itConnectedBody.begin(&t_tree);
             itConnectedBody != itConnectedBody.end();
             ++itConnectedBody) {
            
            /* get the id for this body and store the reference using that id */
            std::string strConnectedBodyId;
            GetNodeAttribute(*itConnectedBody, "id", strConnectedBodyId);
            
            m_tConnectedBodies.push_back(&cBodyEquippedEntity.GetBody(strConnectedBodyId));
            
            //@todo find a generic way of parsing and storing attributes
            m_vecRotationAxes.push_back(CVector3(0,0,0));
            m_vecRotationPoints.push_back(CVector3(0,0,0));

            GetNodeAttribute(*itConnectedBody, "rotation_axis", m_vecRotationAxes.back());
            GetNodeAttribute(*itConnectedBody, "rotation_point", m_vecRotationPoints.back());
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
   }

   /****************************************/
   /****************************************/

   void CJointEntity::Destroy() {
      CComposableEntity::Destroy();
   }

   /****************************************/
   /****************************************/

   CBodyEntity& CJointEntity::GetConnectedBody(UInt32 un_index) {
      ARGOS_ASSERT(un_index < m_tConnectedBodies.size(),
                   "CJointEquippedEntity::GetConnectedBody(), id=\"" <<
                   GetId() <<
                   "\": index out of bounds: un_index = " <<
                   un_index <<
                   ", m_tConnectedBodies.size() = " <<
                   m_tConnectedBodies.size());
      return *m_tConnectedBodies[un_index];
   }


   void CJointEntity::UpdateComponents() {}

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CJointEntity);

   /****************************************/
   /****************************************/

}
