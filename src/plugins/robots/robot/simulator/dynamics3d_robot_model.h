/**
 * @file <argos3/plugins/robots/robot/simulator/dynamics3d_robot_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_ROBOT_MODEL_H
#define DYNAMICS3D_ROBOT_MODEL_H

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/robots/robot/simulator/robot_entity.h>

namespace argos {

   class CDynamics3DRobotModel : public CDynamics3DModel {

   public:

      CDynamics3DRobotModel(CDynamics3DEngine& c_engine,
                                 CRobotEntity& c_entity);
      virtual ~CDynamics3DRobotModel();
      

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

      virtual const btTransform& GetModelWorldTransform() const {
         //@todo this returns the current model reference location for MoveTo etc
         CBodyEntity cReferenceBody = m_cRobotEntity.GetBodyEquippedEntity().GetReferenceBody();
         
         std::map<std::string, SBodyConfiguration*>::const_iterator it = 
            m_mapRobotBodyConfigurations.find(cReferenceBody.GetId());

         return it->second->m_pcRigidBody->getWorldTransform();
      }

   private:
      //@todo migrate this function to CDynamics3DModel
      // method for internal use only, i.e. by reset, moveto, constructor etc
      void SetModelCoordinates(const CVector3& c_position,
                               const CQuaternion& c_orientation) {}

   private:

      CRobotEntity&           m_cRobotEntity;
      CBodyEquippedEntity&    m_cBodyEquippedEntity;
      CJointEquippedEntity&   m_cJointEquippedEntity;

      struct SBodyConfiguration {
         btBoxShape* m_pcCollisionShape;
         btDefaultMotionState* m_pcMotionState;
         btRigidBody* m_pcRigidBody;
      };
      
      std::map<std::string, SBodyConfiguration*> m_mapRobotBodyConfigurations;
      std::map<std::string, btHingeConstraint*> m_mapRobotConstraints;
   };
}

#endif
