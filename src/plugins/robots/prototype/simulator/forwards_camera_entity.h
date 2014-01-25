/**
 * @file <argos3/plugins/robots/prototype/simulator/forwards_camera_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef FORWARDS_CAMERA_ENTITY_H
#define FORWARDS_CAMERA_ENTITY_H

namespace argos {
   class CForwardsCameraEntity;
   
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/vector2.h>

namespace argos {

   class CForwardsCameraEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CForwardsCameraEntity*> TList;

   public:

      CForwardsCameraEntity(CComposableEntity* pc_parent);

      CForwardsCameraEntity(CComposableEntity* pc_parent,
                            const CRadians& c_field_of_view,
                            Real f_range,
                            UInt32 un_horizontal_resolution,
                            UInt32 un_vertical_resolution);

      virtual ~CForwardsCameraEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      //virtual void Reset() {}

      //virtual void SetEnabled(bool b_enabled) {}

      const CRadians& GetFieldOfView() const {
         return m_cFieldOfView;
      }

      Real GetRange() const {
         return m_fRange;
      }

      UInt32 GetHorizontalResolution() const {
         return m_unHorizontalResolution;
      }

      UInt32 GetVerticalResolution() const {
         return m_unVerticalResolution;
      }

      virtual std::string GetTypeDescription() const {
         return "forwards_camera";
      }

      /* TEMP / TESTING */
      CVector3 SphereCenter;
      Real SphereRadius;

   protected:

      CRadians m_cFieldOfView;
      Real m_fRange;
      UInt32 m_unHorizontalResolution;
      UInt32 m_unVerticalResolution;

   };

   /****************************************/
   /****************************************/

}

#endif
