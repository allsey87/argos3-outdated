/**
 * @file <argos3/plugins/robots/robot/simulator/joint_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef JOINT_ENTITY_H
#define JOINT_ENTITY_H

namespace argos {
   class CJointEntity;
   class CFrameEquippedEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>
//#include <argos3/plugins/robots/robot/simulator/frame_equipped_entity.h>

namespace argos {

   class CJointEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CJointEntity*> TList;

   public:

      template <class T>
      struct SAxisDegreeOfFreedom {
         SAxisDegreeOfFreedom(bool b_unconstrained = false, 
                              const CRange<T>& c_limits = CRange<T>()) :
            m_bUnconstrained(b_unconstrained),  
            m_cLimits(c_limits) {}
         bool m_bUnconstrained;
         CRange<T> m_cLimits;
      };

   public:

      CJointEntity(CComposableEntity* pc_parent);

      CJointEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   bool b_disable_collisions);

      virtual ~CJointEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CFrameEquippedEntity& GetFrameEquippedEntity() {
         return *m_pcFrameEquippedEntity;
      }

      virtual std::string GetTypeDescription() const {
         return "joint";
      }

      bool GetDisableLinkedBodyCollisions() const {
         return m_bDisableCollisions;
      }

      const SAxisDegreeOfFreedom<Real>& GetDofLinearX() const {
         return m_sLinearDofs.m_sX;
      }

      const SAxisDegreeOfFreedom<Real>& GetDofLinearY() const {
         return m_sLinearDofs.m_sY;
      }

      const SAxisDegreeOfFreedom<Real>& GetDofLinearZ() const {
         return m_sLinearDofs.m_sZ;
      }

      const SAxisDegreeOfFreedom<CRadians>& GetDofAngularX() const {
         return m_sAngularDofs.m_sX;
      }

      const SAxisDegreeOfFreedom<CRadians>& GetDofAngularY() const {
         return m_sAngularDofs.m_sY;
      }

      const SAxisDegreeOfFreedom<CRadians>& GetDofAngularZ() const {
         return m_sAngularDofs.m_sZ;
      }

   private:      
      bool m_bDisableCollisions;

      CFrameEquippedEntity* m_pcFrameEquippedEntity;

      struct {
         SAxisDegreeOfFreedom<CRadians> m_sX, m_sY, m_sZ;
      } m_sAngularDofs;
      
      struct {
         SAxisDegreeOfFreedom<Real> m_sX, m_sY, m_sZ;
      } m_sLinearDofs;
      

   };

}

#endif
