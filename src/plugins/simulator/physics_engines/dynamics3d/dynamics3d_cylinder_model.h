/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_CYLINDER_MODEL_H
#define DYNAMICS3D_CYLINDER_MODEL_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DCylinderModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>


namespace argos {

   class CDynamics3DCylinderModel : public CDynamics3DModel {

   public:
      
      CDynamics3DCylinderModel(CDynamics3DEngine& c_engine,
                     CCylinderEntity& c_cylinder);
      virtual ~CDynamics3DCylinderModel();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {

         //fprintf(stderr, "[DEBUG] m_graphicsWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", m_pcCylinderMotionState->m_graphicsWorldTrans.getOrigin().getX(), m_pcCylinderMotionState->m_graphicsWorldTrans.getOrigin().getY(), m_pcCylinderMotionState->m_graphicsWorldTrans.getOrigin().getZ(), m_pcCylinderMotionState->m_graphicsWorldTrans.getRotation().getAxis().getX(), m_pcCylinderMotionState->m_graphicsWorldTrans.getRotation().getAxis().getY(), m_pcCylinderMotionState->m_graphicsWorldTrans.getRotation().getAxis().getZ(), m_pcCylinderMotionState->m_graphicsWorldTrans.getRotation().getAngle() * 57.2957795131f);

         //fprintf(stderr, "[DEBUG] m_centerOfMassOffset: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", m_pcCylinderMotionState->m_centerOfMassOffset.getOrigin().getX(), m_pcCylinderMotionState->m_centerOfMassOffset.getOrigin().getY(), m_pcCylinderMotionState->m_centerOfMassOffset.getOrigin().getZ(), m_pcCylinderMotionState->m_centerOfMassOffset.getRotation().getAxis().getX(), m_pcCylinderMotionState->m_centerOfMassOffset.getRotation().getAxis().getY(), m_pcCylinderMotionState->m_centerOfMassOffset.getRotation().getAxis().getZ(), m_pcCylinderMotionState->m_centerOfMassOffset.getRotation().getAngle() * 57.2957795131f);

         //fprintf(stderr, "[DEBUG] m_startWorldTrans: position = [%.3f, %.3f, %.3f], rotation axis = [%.3f, %.3f, %.3f] & angle = %.3f\n", m_pcCylinderMotionState->m_startWorldTrans.getOrigin().getX(), m_pcCylinderMotionState->m_startWorldTrans.getOrigin().getY(), m_pcCylinderMotionState->m_startWorldTrans.getOrigin().getZ(), m_pcCylinderMotionState->m_startWorldTrans.getRotation().getAxis().getX(), m_pcCylinderMotionState->m_startWorldTrans.getRotation().getAxis().getY(), m_pcCylinderMotionState->m_startWorldTrans.getRotation().getAxis().getZ(), m_pcCylinderMotionState->m_startWorldTrans.getRotation().getAngle() * 57.2957795131f);
      }
      
      virtual const btTransform& GetModelWorldTransform() const {
         return m_pcCylinderRigidBody->getWorldTransform();
      }

      //virtual void CalculateBoundingBox();
      
   private:

      CCylinderEntity&           m_cCylinderEntity;
      
      btCylinderShape*           m_pcCylinderCollisionShape;
      btDefaultMotionState*      m_pcCylinderMotionState;
      btRigidBody*               m_pcCylinderRigidBody;
   };
}

#endif
