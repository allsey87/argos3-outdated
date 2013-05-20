/**
 * @file <argos3/plugins/robots/robot/simulator/body_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef BODY_ENTITY_H
#define BODY_ENTITY_H

namespace argos {
   class CBodyEntity;
   class CPositionalEntity;
   class CGeometry3;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CBodyEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CBodyEntity*> TList;

   public:

      CBodyEntity(CComposableEntity* pc_parent);

      CBodyEntity(CComposableEntity* pc_parent,
                  const std::string& str_id,
                  const CVector3& c_offset_position,
                  const CQuaternion& c_offset_orientation,
                  const CVector3& c_size,
                  Real f_mass);

      virtual ~CBodyEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CPositionalEntity& GetPositionalEntity() {
         return *m_pcPositionalEntity;
      }

      CPositionalEntity& GetOffsetPositionalEntity() {
         return *m_pcOffsetPositionalEntity;
      }   

      virtual std::string GetTypeDescription() const {
         return "body";
      }

      const CVector3 & GetSize() const {
         return m_cSize;
      }

      const CGeometry3& GetGeometry() const {
         return *m_pcGeometry;
      }

      Real GetMass() const {
         return m_fMass;
      }

   private:

      CPositionalEntity* m_pcPositionalEntity;
      CPositionalEntity* m_pcOffsetPositionalEntity;

      CGeometry3* m_pcGeometry;
      Real m_fMass;
   };

}

#endif
