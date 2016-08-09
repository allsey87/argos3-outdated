/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.cpp>
 *
 * @author Haitham Elfaham - <haithamelfaham@gmail.com>
 */

#include "dynamics3d_magnetism_plugin.h"
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

#include <algorithm>


std::ostream& operator<<(std::ostream& stream, const btVector3& bt_vector) {
	stream << "(" << bt_vector.getX() << ", "
				 << bt_vector.getY() << ", " 
				 << bt_vector.getZ() << ")";
	return stream;
}

namespace argos {
   
   /****************************************/
   /****************************************/
   
   void CDynamics3DMagnetismPlugin::Init(TConfigurationNode& t_tree) {           
      //force_constant is calculated for Air medium.. replace with 7.0459388e-13 for vacuum, 5.643797e-11 for water @20 degrees celcius
      GetNodeAttributeOrDefault(t_tree, "force_constant", m_fForceConstant, 7.0500949e-13);
   } 
   
   /****************************************/
   /****************************************/

   void CDynamics3DMagnetismPlugin::RegisterModel(CDynamics3DModel& c_model) {
      CComposableEntity& cComposable = c_model.GetEmbodiedEntity().GetParent();
      if(cComposable.HasComponent("electromagnets")) {
         CElectromagnetEquippedEntity& cElectromagnets = cComposable.GetComponent<CElectromagnetEquippedEntity>("electromagnets");        
         for(UInt32 unBodyIdx = 0;
             unBodyIdx < c_model.GetBodies().size();
             ++unBodyIdx) {
            for(UInt32 unElectromagnetIdx = 0;
                unElectromagnetIdx < cElectromagnets.GetAllElectromagneticBodies().size();
                ++unElectromagnetIdx) {
               if(cElectromagnets.GetElectromagneticBody(unElectromagnetIdx).GetId() == c_model.GetBodies()[unBodyIdx]->GetId()) {
                  /* hack set damping on magnets */
                  c_model.GetBodies()[unBodyIdx]->SetDamping(0.0, 0.999999);
                  /* add magnet to list of magnetic bodies */
                  m_tMagneticBodies.push_back(
                     SMagneticBody(&c_model,
                                   c_model.GetBodies()[unBodyIdx],
                                   &cElectromagnets.GetElectromagnet(unElectromagnetIdx)
                                   
                     )
                  );
               }
            }
         }  
      }
   }

   /****************************************/
   /****************************************/

   struct SRemoveOp : std::unary_function<CDynamics3DMagnetismPlugin::SMagneticBody, bool> {
      SRemoveOp(CDynamics3DModel& c_model) :
         Model(c_model) {}
      bool operator()(const CDynamics3DMagnetismPlugin::SMagneticBody& s_body) {
         return (s_body.Parent == &Model);
      }
      const CDynamics3DModel& Model;
   };

   /****************************************/
   /****************************************/

   void CDynamics3DMagnetismPlugin::UnregisterModel(CDynamics3DModel& c_model) {
      m_tMagneticBodies.erase(
         std::remove_if(m_tMagneticBodies.begin(), m_tMagneticBodies.end(), SRemoveOp(c_model)),
      m_tMagneticBodies.end());
   }
   
   /****************************************/
   /****************************************/
   
   void CDynamics3DMagnetismPlugin::Update() {
      
      // add drag force with respect to angular velocity
      for(SMagneticBody::TListIterator itBody = m_tMagneticBodies.begin();
          itBody != m_tMagneticBodies.end();
          ++itBody) {
         //btVector3 cAngularVelocity = itBody->Body->m_pcRigidBody->getAngularVelocity();
         //itBody->Body->ApplyTorque(-0.1 * cAngularVelocity);
      }
      
      for(SMagneticBody::TListIterator itMainBody = m_tMagneticBodies.begin();
          itMainBody != m_tMagneticBodies.end() - 1;
          ++itMainBody) {
         for(SMagneticBody::TListIterator itSubBody = itMainBody + 1;
             itSubBody != m_tMagneticBodies.end();
             ++itSubBody) {
            
            //perform Barnes-hut algorithm            
            const btVector3& cBodyPositionalCoordinatesSub = itSubBody->Body->GetRigidBodyTransform().getOrigin();            
            const btVector3& cBodyPositionalCoordinatesMaster = itMainBody->Body->GetRigidBodyTransform().getOrigin();            
                      
            const btVector3 cSeparationRangeMaster = (cBodyPositionalCoordinatesMaster) - (cBodyPositionalCoordinatesSub);
            const btScalar fSeparationModulusMaster = cBodyPositionalCoordinatesMaster.distance(cBodyPositionalCoordinatesSub);

            // hack - don't consider magnets more than 4cm apart
            if(fSeparationModulusMaster > 0.040) {
               continue;
            }

            const btVector3 cNormalizedSeparationRangeMaster = (cSeparationRangeMaster) / (fSeparationModulusMaster);

            const btVector3 cSeparationRangeSub = cBodyPositionalCoordinatesSub - cBodyPositionalCoordinatesMaster;
            const btScalar fSeparationModulusSub = cBodyPositionalCoordinatesSub.distance(cBodyPositionalCoordinatesMaster);

            const btVector3 cNormalizedSeparationRangeSub = (cSeparationRangeSub)/(fSeparationModulusSub);

            btTransform cMainBodyRotation = itMainBody->Body->GetRigidBodyTransform();
            cMainBodyRotation.getOrigin() = btVector3(0.0f,0.0f,0.0f);
            
            btTransform cSubBodyRotation = itSubBody->Body->GetRigidBodyTransform();
            cSubBodyRotation.getOrigin() = btVector3(0.0f,0.0f,0.0f);

            btVector3 cFieldMainRotated = cMainBodyRotation * ARGoSToBullet(itMainBody->Electromagnet->GetField());
            btVector3 cFieldSubRotated = cSubBodyRotation * ARGoSToBullet(itSubBody->Electromagnet->GetField());

            const btVector3 cCrossProduct1 = cFieldMainRotated.cross(cFieldSubRotated);
            const btVector3 cCrossProduct2 = cFieldMainRotated.cross(cNormalizedSeparationRangeMaster);
            const btVector3 cCrossProduct3 = cFieldSubRotated.cross(cFieldMainRotated);
            const btVector3 cCrossProduct4 = cFieldSubRotated.cross(cNormalizedSeparationRangeSub);

            const btScalar cDotProduct = cFieldMainRotated.dot(cFieldSubRotated);

            const btScalar cDotProduct1 = cFieldMainRotated.dot(cNormalizedSeparationRangeMaster);
            const btScalar cDotProduct2 = cFieldMainRotated.dot(cNormalizedSeparationRangeSub);
            const btScalar cDotProduct3 = cFieldSubRotated.dot(cNormalizedSeparationRangeMaster);
            const btScalar cDotProduct4 = cFieldSubRotated.dot(cNormalizedSeparationRangeSub);

            btVector3 cComponentsTorqueMain = ((3 * cCrossProduct2 * cDotProduct3) - cCrossProduct1) * m_fForceConstant / btPow(fSeparationModulusMaster, 3);      
            btVector3 cComponentsTorqueSub = ((3 * cCrossProduct4 * cDotProduct2) - cCrossProduct3) * m_fForceConstant / btPow(fSeparationModulusSub, 3);
            
            btVector3 cComponentsForceMain = (m_fForceConstant / btPow(fSeparationModulusMaster, 4)) * (
               (-15 * cNormalizedSeparationRangeMaster * (cDotProduct1 * cDotProduct3)) +
               (3 * cNormalizedSeparationRangeMaster * cDotProduct) +
               (3 * (cFieldMainRotated * cDotProduct3 + cFieldSubRotated * cDotProduct1))
            );

            btVector3 cComponentsForceSub = (m_fForceConstant/ btPow(fSeparationModulusSub, 4)) * (
               (-15 * cNormalizedSeparationRangeSub * (cDotProduct4 * cDotProduct2)) +
               (3 * cNormalizedSeparationRangeSub * cDotProduct) +
               (3 * (cFieldSubRotated * cDotProduct2 + cFieldMainRotated * cDotProduct4))
            );
/*
            std::cerr << "Pos Main: " << cBodyPositionalCoordinatesMaster << std::endl;
            std::cerr << "Pos Sub: " << cBodyPositionalCoordinatesSub << std::endl;

            std::cerr << "Torque: " << cComponentsTorqueMain << std::endl;
            std::cerr << "Torque: " << cComponentsTorqueSub << std::endl;
*/
            itMainBody->Body->ApplyForce(btVector3(cComponentsForceMain.getX(), 
                                                   cComponentsForceMain.getY(), 
                                                   cComponentsForceMain.getZ()));
            itMainBody->Body->ApplyTorque(btVector3(cComponentsTorqueMain.getX(),
                                                    cComponentsTorqueMain.getY(),
                                                    cComponentsTorqueMain.getZ()));
            itSubBody->Body->ApplyForce(btVector3(cComponentsForceSub.getX(),
                                                  cComponentsForceSub.getY(), 
                                                  cComponentsForceSub.getZ()));
            itSubBody->Body->ApplyTorque(btVector3(cComponentsTorqueSub.getX(), 
                                                   cComponentsTorqueSub.getY(),
                                                   cComponentsTorqueSub.getZ()));               
         }
      }

   }
   /****************************************/
   /****************************************/
   
   REGISTER_DYN3D_PHYSICS_PLUGIN(CDynamics3DMagnetismPlugin,
                                 "magnetism",
                                 "Haitham Elfaham [haithamelfaham@gmail.com]",
                                 "1.0",
                                 "A plugin developed for testing the dynamics3d plugin mechanism",
                                 "This plugin applies a specified force to all bodies in the"
                                 "physics engine that it is applied to. The plugin has been "
                                 "developed for testing the dynamics3d plugin mechanism",
                                 "Under development");
}
