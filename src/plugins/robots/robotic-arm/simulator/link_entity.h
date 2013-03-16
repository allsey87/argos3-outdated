/**
 * @file <argos3/plugins/robots/robotic-arm/simulator/led_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef LINK_ENTITY_H
#define LINK_ENTITY_H

namespace argos {
   class CLinkEntity;
   class CPositionalEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CLinkEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CLinkEntity*> TList;

   public:

      CLinkEntity(CComposableEntity* pc_parent);

      CLinkEntity(CComposableEntity* pc_parent,
                  const std::string& str_id,
                  const CVector3& c_position,
                  const CQuaternion& c_orientation);

      virtual ~CLinkEntity() {}

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();
      virtual void Destroy();

      virtual void UpdateComponents();

      CPositionalEntity& GetPositionalEntity() {
         return *m_pcPositionalEntity;
      }   


      virtual std::string GetTypeDescription() const {
         /* @todo remove this horrid hack once Id logic is fixed */
         static int cnt = 0;
         std::ostringstream out;
         out << "link[" << cnt++ << "]";
         return out.str().c_str();
         //return "link";
      }

   private:

      CPositionalEntity* m_pcPositionalEntity;

   };

}

#endif
