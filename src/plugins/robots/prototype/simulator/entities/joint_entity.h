/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/joint_entity.h>
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

namespace argos {

   class CJointEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CJointEntity*> TList;

   public:

      template <class T>
      struct SAxisParameters {
         SAxisParameters(bool b_unconstrained = false,
                         const CRange<T>& c_limits = CRange<T>(),
                         bool b_actuator_enabled = false,
                         Real f_actuator_force = 0.0f,
                         Real f_actuator_target_velocity = 0.0f) :
            m_bUnconstrained(b_unconstrained),  
            m_cLimits(c_limits),
            m_bActuatorEnabled(b_actuator_enabled),
            m_fActuatorForce(f_actuator_force),
            m_fActuatorTargetVelocity(f_actuator_target_velocity) {}
         bool m_bUnconstrained;
         CRange<T> m_cLimits; //@todo rename to range
         bool m_bActuatorEnabled;
         Real m_fActuatorForce;
         Real m_fActuatorTargetVelocity;
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

      // @todo remame to SetActuatorParametersLinearX etc

      void SetActuatorParametersLinearX(Real f_target_velocity,
                           bool b_enabled) {
         m_sLinearAxes.m_sX.m_fActuatorTargetVelocity = f_target_velocity;
         m_sLinearAxes.m_sX.m_bActuatorEnabled = b_enabled;
      }

      void SetActuatorParametersLinearY(Real f_target_velocity,
                           bool b_enabled) {
         m_sLinearAxes.m_sY.m_fActuatorTargetVelocity = f_target_velocity;
         m_sLinearAxes.m_sY.m_bActuatorEnabled = b_enabled;
      }

      void SetActuatorParametersLinearZ(Real f_target_velocity,
                           bool b_enabled) {
         m_sLinearAxes.m_sZ.m_fActuatorTargetVelocity = f_target_velocity;
         m_sLinearAxes.m_sZ.m_bActuatorEnabled = b_enabled;
      }

      void SetActuatorParametersAngularX(Real f_target_velocity,
                           bool b_enabled) {
         m_sAngularAxes.m_sX.m_fActuatorTargetVelocity = f_target_velocity;
         m_sAngularAxes.m_sX.m_bActuatorEnabled = b_enabled;
      }

      void SetActuatorParametersAngularY(Real f_target_velocity,
                           bool b_enabled) {
         m_sAngularAxes.m_sY.m_fActuatorTargetVelocity = f_target_velocity;
         m_sAngularAxes.m_sY.m_bActuatorEnabled = b_enabled;
      }

      void SetActuatorParametersAngularZ(Real f_target_velocity,
                           bool b_enabled) {
         m_sAngularAxes.m_sZ.m_fActuatorTargetVelocity = f_target_velocity;
         m_sAngularAxes.m_sZ.m_bActuatorEnabled = b_enabled;
      }


      const SAxisParameters<Real>& GetAxisLinearX() const { // rename to GetAxisLinearX etc
         return m_sLinearAxes.m_sX;
      }

      const SAxisParameters<Real>& GetAxisLinearY() const {
         return m_sLinearAxes.m_sY;
      }

      const SAxisParameters<Real>& GetAxisLinearZ() const {
         return m_sLinearAxes.m_sZ;
      }

      const SAxisParameters<CRadians>& GetAxisAngularX() const {
         return m_sAngularAxes.m_sX;
      }

      const SAxisParameters<CRadians>& GetAxisAngularY() const {
         return m_sAngularAxes.m_sY;
      }

      const SAxisParameters<CRadians>& GetAxisAngularZ() const {
         return m_sAngularAxes.m_sZ;
      }

      const CQuaternion& GetJointRotation() const {
         return m_cJointRotation;
      }

      const CVector3& GetJointTranslation() const {
         return m_cJointTranslation;
      }

      /*
        This method is called from a physics engine to update
        the value of joints current rotation, it does not control the joint 
       */
      void SetJointRotation(const CQuaternion& c_rotation) {
         m_cJointRotation = c_rotation;
      }

      /*
        This method is called from a physics engine to update
        the value of joints current translation, it does not control the joint 
       */
      void SetJointTranslation(const CVector3& c_translation) {
         m_cJointTranslation = c_translation;
      }

            
   private:      
      bool m_bDisableCollisions;

      CFrameEquippedEntity* m_pcFrameEquippedEntity;

      CQuaternion m_cJointRotation;

      CVector3 m_cJointTranslation;

      struct {
         SAxisParameters<CRadians> m_sX, m_sY, m_sZ;
      } m_sAngularAxes;
      
      struct {
         SAxisParameters<Real> m_sX, m_sY, m_sZ;
      } m_sLinearAxes;
      

   };

}

#endif
