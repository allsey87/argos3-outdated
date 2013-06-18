/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/sphere_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef SPHERE_ENTITY_H
#define SPHERE_ENTITY_H

namespace argos {
   class CSphereEntity;
   class CEmbodiedEntity;
   class CLEDEquippedEntity;
}

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>

namespace argos {

   class CSphereEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      CSphereEntity();

      CSphereEntity(const std::string& str_id,
                      const CVector3& c_position,
                      const CQuaternion& c_orientation,
                      bool b_movable,
                      Real f_radius,
                      Real f_mass = 1.0f);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline CLEDEquippedEntity& GetLEDEquippedEntity() {
         return *m_pcLEDEquippedEntity;
      }

      inline Real GetRadius() const {
         return m_fRadius;
      }

      inline void SetRadius(Real c_radius) {
         m_fRadius = c_radius;
      }

      inline Real GetMass() const {
         return m_fMass;
      }

      inline void SetMass(Real f_mass) {
         m_fMass = f_mass;
      }

      virtual std::string GetTypeDescription() const {
         return "sphere";
      }

   private:

      CEmbodiedEntity*      m_pcEmbodiedEntity;
      CLEDEquippedEntity*   m_pcLEDEquippedEntity;
      Real                  m_fRadius;
      Real                  m_fMass;

   };

}

#endif
