/**
 * @file <argos3/plugins/robots/prototype/simulator/entities/camera_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERA_ENTITY_H
#define CAMERA_ENTITY_H

namespace argos {
   class CCameraEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/utility/math/vector2.h>

namespace argos {

   class CCameraEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CCameraEntity*> TList;

   public:

      CCameraEntity(CComposableEntity* pc_parent);

      CCameraEntity(CComposableEntity* pc_parent,
                            const CRadians& c_field_of_view,
                            Real f_range,
                            UInt32 un_horizontal_resolution,
                            UInt32 un_vertical_resolution);

      virtual ~CCameraEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      //virtual void Reset() {}

      //virtual void SetEnabled(bool b_enabled) {}

      const CRadians& GetFieldOfView() const {
         return m_cFieldOfView;
      }

      const CRadians& GetRoll() const {
         return m_cRoll;
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
         return "camera";
      }

   protected:

      CRadians m_cFieldOfView;
      CRadians m_cRoll;
      Real m_fRange;
      UInt32 m_unHorizontalResolution;
      UInt32 m_unVerticalResolution;

   };

   /****************************************/
   /****************************************/

}

#endif
