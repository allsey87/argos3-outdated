/**
 * @file <argos3/plugins/robots/robot/simulator/frame_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef FRAME_ENTITY_H
#define FRAME_ENTITY_H

namespace argos {
   class CFrameEntity;
   class CBodyEntity;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/simulator/entity/composable_entity.h>
//#include <argos3/plugins/robots/robot/simulator/body_entity.h>

namespace argos {

   class CFrameEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CFrameEntity*> TList;

   public:

      CFrameEntity(CComposableEntity* pc_parent);

      CFrameEntity(CComposableEntity* pc_parent,
                   const std::string& str_id,
                   CBodyEntity* pc_body,
                   const CVector3& c_position,
                   const CQuaternion& c_orientation);
      
      virtual ~CFrameEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset();

      virtual void Destroy();
    
      virtual void UpdateComponents();

      const CBodyEntity& GetBodyEntity() const {
         return *m_pcBodyEntity;
      }

      CPositionalEntity& GetPositionalEntity() {
         return *m_pcPositionalEntity;
      }
      
      virtual std::string GetTypeDescription() const {
         return "frame";
      }

   private:      
      CBodyEntity* m_pcBodyEntity;
      CPositionalEntity* m_pcPositionalEntity;
   };

}

#endif
