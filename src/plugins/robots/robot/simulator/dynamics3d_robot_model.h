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

   protected:

      virtual btTransform GetModelCoordinates() const;

   private:

      CRobotEntity&           m_cRobotEntity;
      CBodyEquippedEntity&    m_cBodyEquippedEntity;
      CJointEquippedEntity&   m_cJointEquippedEntity;
     
      //std::map<std::string, btHingeConstraint> m_mapRobotConstraints;
   };
}

#endif
