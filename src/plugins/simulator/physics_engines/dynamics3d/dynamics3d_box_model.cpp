/*sudo*
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_box_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBoxModel::CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                                            CBoxEntity& c_box) :
      CDynamics3DModel(c_engine, c_box.GetEmbodiedEntity()),
      m_cBoxEntity(c_box) {
      
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcBoxCollisionShape = new btBoxShape(btVector3(c_box.GetSize().GetX() * 0.5f,
                                                       c_box.GetSize().GetZ() * 0.5f, 
                                                       c_box.GetSize().GetY() * 0.5f));
      

      btTransform boxGeo(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
                         btVector3(0.0f, -c_box.GetSize().GetZ() * 0.5f, 0.0f));

      Real fMass = c_box.GetEmbodiedEntity().IsMovable() ? c_box.GetMass() : 0.0f;
 
      m_vecLocalBodies.push_back(
         CDynamics3DBody::TNamedElement("box", new CDynamics3DBody(m_pcBoxCollisionShape,
                                                                   btTransform::getIdentity(),
                                                                   boxGeo,
                                                                   fMass)));
      /* move the model to the specified coordinates */
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetInitOrientation()),
                                      ARGoSToBullet(GetEmbodiedEntity().GetInitPosition())));
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DBoxModel::~CDynamics3DBoxModel() {
      delete m_pcBoxCollisionShape;
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DBoxModel::UpdateEntityStatus() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         const btTransform& cUpdateTransform = GetModelCoordinates();
         
         GetEmbodiedEntity().SetPosition(BulletToARGoS(cUpdateTransform.getOrigin()));
         GetEmbodiedEntity().SetOrientation(BulletToARGoS(cUpdateTransform.getRotation()));

         /* Update components */
         m_cBoxEntity.UpdateComponents();
     }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DBoxModel::GetModelCoordinates() const {
      return m_vecLocalBodies[0]->GetMotionStateTransform();
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxModel);

}
