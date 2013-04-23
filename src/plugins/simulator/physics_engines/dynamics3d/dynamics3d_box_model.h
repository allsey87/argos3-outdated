/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_BOX_MODEL_H
#define DYNAMICS3D_BOX_MODEL_H

namespace argos {
   class CDynamics3DEngine;
   class CDynamics3DBoxModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/entities/box_entity.h>


namespace argos {

   class CDynamics3DBoxModel : public CDynamics3DModel {

   public:
      
      CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                     CBoxEntity& c_box);
      virtual ~CDynamics3DBoxModel();

      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus() {}
 
   protected:
      
      virtual btTransform GetModelCoordinates() const;
      
   private:

      CBoxEntity&                m_cBoxEntity;
      
      btBoxShape*                m_pcBoxCollisionShape;
   };
}

#endif
