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
     
      
      class CBoxShapeManager {
         public:
            btBoxShape* RequestBoxShape(const btVector3& c_half_extents);
            void ReleaseBoxShape(const btBoxShape* pc_release);
         private:
            struct CResource {
               CResource(const btVector3& c_half_extents, btBoxShape* c_shape);
               btVector3 m_cHalfExtents;
               btBoxShape* m_cShape;
               UInt32 m_unInUseCount;
            };
            std::vector<CResource> m_vecResources;
      };
      
      class CCylinderShapeManager {
         public:
            btCylinderShape* RequestCylinderShape(const btVector3& c_half_extents);
            void ReleaseCylinderShape(const btCylinderShape* pc_release);
         private:
            struct CResource {
               CResource(const btVector3& c_half_extents, btCylinderShape* c_shape);
               btVector3 m_cHalfExtents;
               btCylinderShape* m_cShape;
               UInt32 m_unInUseCount;
            };
            std::vector<CResource> m_vecResources;
      };

      class CSphereShapeManager {
         public:
            btSphereShape* RequestSphereShape(Real f_radius);
            void ReleaseSphereShape(const btSphereShape* pc_release);
         private:
            struct CResource {
               CResource(Real f_radius, btSphereShape* c_shape);
               Real m_fRadius;
               btSphereShape* m_cShape;
               UInt32 m_unInUseCount;
            };
            std::vector<CResource> m_vecResources;
      };
      
      static CBoxShapeManager m_cBoxShapeManager;
      static CCylinderShapeManager m_cCylinderShapeManager;
      static CSphereShapeManager m_cSphereShapeManager;
   };
}

#endif
