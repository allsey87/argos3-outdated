/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/body_equipped_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef BODY_EQUIPPED_ENTITY_H
#define BODY_EQUIPPED_ENTITY_H

namespace argos {
   class CBodyEquippedEntity;
   class CBodyEntity;
}

#include <argos3/plugins/robots/robot/simulator/body_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <map>

namespace argos {

   class CBodyEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::map<std::string, CBodyEquippedEntity*> TMap;

   public:

      CBodyEquippedEntity(CComposableEntity* pc_parent);

      CBodyEquippedEntity(CComposableEntity* pc_parent,
                          const std::string& str_id);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Update() {}
      /*
      void AddBody(const CVector3& c_position,
                   const CQuaternion& c_orientation,
                   const CVector3& c_size,
                   Real f_mass);
      */
      CBodyEntity& GetBody(UInt32 un_index);

      CBodyEntity& GetReferenceBody() {
         return *m_pcReferenceBody;
      }

      inline CBodyEntity::TList& GetAllBodies() {
         return m_tBodies;
      }

      virtual std::string GetTypeDescription() const {
         return "bodies";
      }

   protected:

      virtual void UpdateComponents();

   protected:

      CBodyEntity::TList m_tBodies;

      CBodyEntity * m_pcReferenceBody;
   };

}

#endif
