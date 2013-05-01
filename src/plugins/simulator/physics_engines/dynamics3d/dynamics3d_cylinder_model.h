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
      virtual void UpdateFromEntityStatus() {}
      
   protected:

      virtual btTransform GetModelCoordinates() const;
      
   private:

      CCylinderEntity&           m_cCylinderEntity;
      
      btCylinderShape*           m_pcCylinderCollisionShape;
   };
}

#endif
