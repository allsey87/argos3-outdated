/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef JOINT_ENTITY_H
#define JOINT_ENTITY_H

namespace argos {
   class CJointEntity;
   class CBodyEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/robots/robot/simulator/body_equipped_entity.h>

namespace argos {

   class CJointEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CJointEntity*> TList;

   public:
      enum EJointType {
         ROTARY,
         PRISMATIC,
         PIVOT
      };


   public:

      CJointEntity(CComposableEntity* pc_parent);

      CJointEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   EJointType e_joint_type,
                   bool b_disable_collisions,
                   const CBodyEntity::TList& t_connected_bodies,
                   
                   const std::vector<CVector3>& vec_rotation_axes,
                   const std::vector<CVector3>& vec_rotation_points);

      virtual ~CJointEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CBodyEntity& GetConnectedBody(UInt32 un_index);
            
      CBodyEntity::TList& GetAllConnectedBodies() {
         return m_tConnectedBodies;
      }

      virtual std::string GetTypeDescription() const {
         return "joint";
      }



   private:      
      EJointType m_eJointType;

      bool m_bDisableCollisions;

      CBodyEntity::TList m_tConnectedBodies;

   public:

      //@todo find a generic way of parsing and storing attributes
      std::vector<CVector3> m_vecRotationAxes;
      std::vector<CVector3> m_vecRotationPoints;
   };

}

#endif
