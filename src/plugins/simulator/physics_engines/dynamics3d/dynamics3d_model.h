/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_MODEL_H
#define DYNAMICS3D_MODEL_H

namespace argos {
   class  CDynamics3DEngine;
}

#include <argos3/core/simulator/physics_engine/physics_model.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_joint.h>

#include <tr1/unordered_map>

namespace argos {

   /****************************************/
   /****************************************/
   
   CVector3 BulletToARGoS(const btVector3& c_bt_vector); /* {
      Real c_bt_vector_x = c_bt_vector.getX();
      Real c_bt_vector_y = c_bt_vector.getY();
      Real c_bt_vector_z = c_bt_vector.getZ();

      fprintf(stderr, "BulletToARGoS : btVector3 = [%f, %f, %f]\n", c_bt_vector.getX(), c_bt_vector.getY(), c_bt_vector.getZ());
      
      fprintf(stderr, "c_bt_vector_x = %f\n", c_bt_vector_x);
      fprintf(stderr, "c_bt_vector_y = %f\n", c_bt_vector_y);
      fprintf(stderr, "c_bt_vector_z = %f\n", c_bt_vector_z);
      

      CVector3 c_a_vector(c_bt_vector_x, -c_bt_vector_z, c_bt_vector_y);

      Real c_a_vector_x = c_a_vector.GetX();
      Real c_a_vector_y = c_a_vector.GetY();
      Real c_a_vector_z = c_a_vector.GetZ();

      fprintf(stderr, "BulletToARGoS : CVector3 = [%f, %f, %f]\n", c_a_vector.GetX(), c_a_vector.GetY(), c_a_vector.GetZ());

      
      fprintf(stderr, "c_a_vector_x = %f\n", c_a_vector_x);
      fprintf(stderr, "c_a_vector_y = %f\n", c_a_vector_y);
      fprintf(stderr, "c_a_vector_z = %f\n", c_a_vector_z);
      

      return c_a_vector;

      //return CVector3(c_bt_vector.getX(), -c_bt_vector.getZ(), c_bt_vector.getY());
      } */
   
   btVector3 ARGoSToBullet(const CVector3& c_a_vector); /* {
      Real c_a_vector_x = c_a_vector.GetX();
      Real c_a_vector_y = c_a_vector.GetY();
      Real c_a_vector_z = c_a_vector.GetZ();

      fprintf(stderr, "ARGoSToBullet : CVector3 = [%f, %f, %f]\n", c_a_vector.GetX(), c_a_vector.GetY(), c_a_vector.GetZ());

      
      fprintf(stderr, "c_a_vector_x = %f\n", c_a_vector_x);
      fprintf(stderr, "c_a_vector_y = %f\n", c_a_vector_y);
      fprintf(stderr, "c_a_vector_z = %f\n", c_a_vector_z);
      

      btVector3 c_bt_vector(c_a_vector_x, c_a_vector_z, -c_a_vector_y);

      Real c_bt_vector_x = c_bt_vector.getX();
      Real c_bt_vector_y = c_bt_vector.getY();
      Real c_bt_vector_z = c_bt_vector.getZ();
       
      fprintf(stderr, "ARGoSToBullet : btVector3 = [%f, %f, %f]\n", c_bt_vector.getX(), c_bt_vector.getY(), c_bt_vector.getZ());

      
      fprintf(stderr, "c_bt_vector_x = %f\n", c_bt_vector_x);
      fprintf(stderr, "c_bt_vector_y = %f\n", c_bt_vector_y);
      fprintf(stderr, "c_bt_vector_z = %f\n", c_bt_vector_z);
      

      return c_bt_vector;
      //return btVector3(c_a_vector.GetX(), c_a_vector.GetZ(), -c_a_vector.GetY());
      } */
   
   CQuaternion BulletToARGoS(const btQuaternion& c_bt_quaternion);
   
   btQuaternion ARGoSToBullet(const CQuaternion& c_a_quaternion);
   
   /****************************************/
   /****************************************/

   class CDynamics3DModel : public CPhysicsModel {

   public:

      typedef std::vector<CDynamics3DModel*> TVector;

   public:

      CDynamics3DModel(CDynamics3DEngine& c_engine,
                       CEmbodiedEntity& c_entity,
                       const std::string& str_id) :
         CPhysicsModel(c_engine, c_entity),
         m_cEngine(c_engine),
         m_strId(str_id) {}
      virtual ~CDynamics3DModel();

      virtual bool MoveTo(const CVector3& c_position,
                          const CQuaternion& c_orientation,
                          bool b_check_only = false);

      virtual void Reset();
      
      virtual void UpdateEntityStatus() = 0;

      virtual void UpdateFromEntityStatus() = 0;

      const std::string& GetId() const {
         return m_strId;
      }
      
      inline CDynamics3DBody::TVector& GetBodies() {
         return m_vecLocalBodies;
      }
      inline const CDynamics3DBody::TVector& GetBodies() const {
         return m_vecLocalBodies;
      }
      
      inline CDynamics3DJoint::TVector& GetJoints() {
         return m_vecLocalJoints;
      }
      inline const CDynamics3DJoint::TVector& GetJoints() const {
         return m_vecLocalJoints;
      }

      virtual void CalculateBoundingBox();

      virtual bool IsCollidingWithSomething() const;

      virtual bool CheckIntersectionWithRay(Real& f_t_on_ray,
                                            const CRay3& c_ray) const;

   protected:

      virtual btTransform GetModelCoordinates() const = 0;

      virtual void SetModelCoordinates(const btTransform& c_coordinates);

   protected:

      CDynamics3DEngine&      m_cEngine;

      CDynamics3DBody::TVector m_vecLocalBodies;
      CDynamics3DJoint::TVector m_vecLocalJoints;

      std::string m_strId;

   };
   
   /****************************************/
   /****************************************/

   bool operator==(const CDynamics3DModel* pc_dyn3d_model, const std::string& str_id);

   /****************************************/
   /****************************************/

}

#endif
