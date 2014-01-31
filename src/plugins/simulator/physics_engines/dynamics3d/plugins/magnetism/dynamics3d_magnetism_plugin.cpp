/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.cpp>
 *
 * @author Haitham Elfaham - <haithamelfaham@gmail.com>
 */

#include "dynamics3d_magnetism_plugin.h"
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

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
         m_tModels.push_back(SModel(&c_model, &cElectromagnets));
         
         UInt32 unBodyIdx, unElectromagnetIdx;
         
         for(unBodyIdx = 0;
             unBodyIdx < c_model.GetBodies().size();
             ++unBodyIdx) {
            for(unElectromagnetIdx = 0;
                unElectromagnetIdx < cElectromagnets.GetAllElectromagneticBodies().size();
                ++unElectromagnetIdx) {
               if(cElectromagnets.GetElectromagneticBody(unElectromagnetIdx).GetId() == c_model.GetBodies()[unBodyIdx]->GetId()) {
                  m_tModels.back().BodyIndices.push_back(unBodyIdx);
                  m_tModels.back().ElectromagnetIndices.push_back(unElectromagnetIdx);
               }
            }
         }  
      }
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DMagnetismPlugin::UnregisterModel(CDynamics3DModel& c_model) {
      for(SModel::TList::iterator itModel = m_tModels.begin(); itModel != m_tModels.end(); ++itModel) {
         if(itModel->Model == &c_model) {
            m_tModels.erase(itModel);
         }
      }
   }
   
   /****************************************/
   /****************************************/
   
   void CDynamics3DMagnetismPlugin::Update() {
      /*

      SMagneticBody MainBody;
      SMagneticBody SubBody;
      CVector3 cMainBodyField;
      CVector3 cSubBodyField;
      
      for(std::vector<SMagneticBody>::iterator itMainBody = m_vecMagneticBodies.begin();
          itMainBody != m_vecMagneticBodies.end()-1;
          ++itMainBody) {	         
         for(std::vector<SMagneticBody>::iterator itSubBody = itMainBody+1;
             itSubBody != m_vecMagneticBodies.end();
             ++itSubBody) {		        
            
            //perform Barnes-hut algorithm            
            const btVector3 &cBodyPositionalCoordinatesSub = (*itSubBody).Body->GetRigidBodyTransform().getOrigin();            
            const btVector3 &cBodyPositionalCoordinatesMaster = (*itMainBody).Body->GetRigidBodyTransform().getOrigin();            
            btVector3 cFieldMain= (*itMainBody).Field;
            btVector3 cFieldSub=  (*itSubBody).Field;            
            cFieldMain=cFieldMain.rotate(itMainBody->Body->GetRigidBodyTransform().getRotation().getAxis(),
                                         itMainBody->Body->GetRigidBodyTransform().getRotation().getAngle());
            cFieldSub=cFieldSub.rotate(itSubBody->Body->GetRigidBodyTransform().getRotation().getAxis(),
                                       itSubBody->Body->GetRigidBodyTransform().getRotation().getAngle());           
            const btVector3 cSeparationRangeMaster=(cBodyPositionalCoordinatesMaster)-(cBodyPositionalCoordinatesSub);
            const btScalar fSeparationModulusMaster= cBodyPositionalCoordinatesMaster.distance(cBodyPositionalCoordinatesSub);
            const btVector3 cNormalizedSeparationRangeMaster=(cSeparationRangeMaster)/(fSeparationModulusMaster);            
            const btVector3 cSeparationRangeSub=(cBodyPositionalCoordinatesSub)-(cBodyPositionalCoordinatesMaster);
            const btScalar fSeparationModulusSub= cBodyPositionalCoordinatesSub.distance(cBodyPositionalCoordinatesMaster);
            const btVector3 cNormalizedSeparationRangeSub=(cSeparationRangeSub)/(fSeparationModulusSub);     

       
            const btVector3 cCrossProduct1=cFieldMain.cross(cFieldSub);
            const btVector3 cCrossProduct2=cFieldMain.cross(cNormalizedSeparationRangeMaster);
            const btVector3 cCrossProduct3=cFieldSub.cross(cFieldMain);
            const btVector3 cCrossProduct4=cFieldSub.cross(cNormalizedSeparationRangeSub);
            const btScalar cDotProduct=cFieldMain.dot(cFieldSub);
            const btScalar cDotProduct1=cFieldMain.dot(cNormalizedSeparationRangeSub);
            const btScalar cDotProduct2=cFieldSub.dot(cNormalizedSeparationRangeSub);
            const btScalar cDotProduct3=cFieldSub.dot(cNormalizedSeparationRangeMaster);
            const btScalar cDotProduct4=cFieldMain.dot(cNormalizedSeparationRangeMaster);			               
            btVector3 cComponentsTorqueMain=((3*cCrossProduct2*cDotProduct3)-(cCrossProduct1))*(m_fForceConstant/(btPow(fSeparationModulusMaster,3)));		
            btVector3 cComponentsTorqueSub=((3*cCrossProduct4*cDotProduct1)-(cCrossProduct3))*(m_fForceConstant/(btPow(fSeparationModulusSub,3))); 

            btVector3 cComponentsForceMain= (m_fForceConstant/ btPow(fSeparationModulusMaster,4))* ((-15*cNormalizedSeparationRangeMaster*(cDotProduct1*cDotProduct2)) +(3*cNormalizedSeparationRangeMaster*cDotProduct) +(3*(cFieldMain*(cDotProduct2)+cFieldSub*(cDotProduct1))));
            btVector3 cComponentsForceSub= (m_fForceConstant/ btPow(fSeparationModulusSub,4))* ((-15*cNormalizedSeparationRangeSub*(cDotProduct3*cDotProduct4)) +(3*cNormalizedSeparationRangeSub*cDotProduct) +(3*(cFieldSub*(cDotProduct4)+cFieldMain*(cDotProduct3))));           
            itMainBody->Body->ApplyForce(btVector3(cComponentsForceMain.getX(), 
                                                   cComponentsForceMain.getY(), 
                                                   cComponentsForceMain.getZ()));

      */



      /*

            itMainBody->Body->ApplyTorque(btVector3(cComponentsTorqueMain.getX(),
                                                    cComponentsTorqueMain.getY(), 
                                                    cComponentsTorqueMain.getZ()));            
            itSubBody->Body->ApplyForce(btVector3(cComponentsForceSub.getX(),
                                                  cComponentsForceSub.getY(), 
                                                  cComponentsForceSub.getZ()));
            itSubBody->Body->ApplyTorque(btVector3(cComponentsTorqueSub.getX(), 
                                                   cComponentsTorqueSub.getY(),
                                                   cComponentsTorqueSub.getZ()));


  fprintf(stderr, "Total Torque on Main is [%.3f %.3f %.3f]\n",                                  
                    cComponentsTorqueMain.getX(),
                    cComponentsTorqueMain.getY(),
                    cComponentsTorqueMain.getZ());

	              fprintf(stderr, "Total Torque on Sub  is [%.3f %.3f %.3f]\n",                                  
                cComponentsTorqueSub.getX(),
                cComponentsTorqueSub.getY(),
                cComponentsTorqueSub.getZ());

            /*fprintf(stderr, "Total Force on Main is [%.3f %.3f %.3f]\n",                                  
                    cComponentsForceMain.getX(),
                    cComponentsForceMain.getY(),
                    cComponentsForceMain.getZ());
 fprintf(stderr, "Total FOrce on Sub is [%.3f %.3f %.3f]\n",                                  
                    cComponentsForceSub.getX(),
                    cComponentsForceSub.getY(),
                    cComponentsForceSub.getZ());



               
                         fprintf(stderr, "Dot products are [%.3f %.3f %.3f %.3f %.3f]\n",                                 
                cDotProduct,
                cDotProduct1,
                cDotProduct2,
                cDotProduct3,
               cDotProduct4);

     fprintf(stderr, "fields  are [%.3f %.3f %.3f %.3f %.3f %.3f]\n",                                 
             cFieldSub.getX(),
             cFieldSub.getY(),
             cFieldSub.getZ(),
             cFieldMain.getX(),
             cFieldMain.getY(),
             cFieldMain.getZ());


     fprintf(stderr, "crossproducts  are [%.3f %.3f %.3f %.3f %.3f %.3f]\n",                                 
             cCrossProduct1.getX(),
             cCrossProduct1.getY(),
             cCrossProduct1.getZ(),
             cCrossProduct2.getX(),
             cCrossProduct2.getY(),
             cCrossProduct2.getZ());

  fprintf(stderr, "crossproducts  are [%.3f %.3f %.3f %.3f %.3f %.3f]\n",                                 
             cCrossProduct3.getX(),
             cCrossProduct3.getY(),
             cCrossProduct3.getZ(),
             cCrossProduct4.getX(),
             cCrossProduct4.getY(),
             cCrossProduct4.getZ());
  fprintf(stderr, "Normalized separations  are [%.3f %.3f %.3f %.3f %.3f %.3f]\n",                                 
          cNormalizedSeparationRangeMaster.getX(),
          cNormalizedSeparationRangeMaster.getY(),
             cNormalizedSeparationRangeMaster.getZ(),
             cNormalizedSeparationRangeSub.getX(),
             cNormalizedSeparationRangeSub.getY(),
             cNormalizedSeparationRangeSub.getZ());

  fprintf(stderr, "separations  are [%.3f %.3f %.3f %.3f %.3f %.3f]\n",                                 
          cSeparationRangeMaster.getX(),
          cSeparationRangeMaster.getY(),
             cSeparationRangeMaster.getZ(),
             cSeparationRangeSub.getX(),
             cSeparationRangeSub.getY(),
             cSeparationRangeSub.getZ());

 fprintf(stderr, "checks  are [%.3f %.3f %.3f ]\n",                                 
         ( ((*itSubBody).Body->GetRigidBodyTransform().getOrigin().cross(cComponentsForceSub))-( (*itMainBody).Body->GetRigidBodyTransform().getOrigin().cross(cComponentsForceMain))).getX(),
         ( ((*itSubBody).Body->GetRigidBodyTransform().getOrigin().cross(cComponentsForceSub))-( (*itMainBody).Body->GetRigidBodyTransform().getOrigin().cross(cComponentsForceMain))).getY(),
      ( ((*itSubBody).Body->GetRigidBodyTransform().getOrigin().cross(cComponentsForceSub))-( (*itMainBody).Body->GetRigidBodyTransform().getOrigin().cross(cComponentsForceMain))).getZ());


  fprintf(stderr, "modulii  are [%.3f %.3f]\n",                                 
          fSeparationModulusSub,
      fSeparationModulusMaster);

  fprintf(stderr, "products of torques are [%.3f %.3f %.3f %.3f %.3f %.3f]\n",                                 
            (cCrossProduct2*cDotProduct3).getX(),
            (cCrossProduct2*cDotProduct3).getY(),
               (cCrossProduct2*cDotProduct3).getZ(),
             (cCrossProduct4*cDotProduct1).getX(),
             (cCrossProduct4*cDotProduct1).getY(),
             (cCrossProduct4*cDotProduct1).getZ()); 
         }
         }*/
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
