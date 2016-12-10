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
#include <argos3/core/utility/math/matrix/squarematrix.h>

namespace argos {

   class CCameraEntity : public CEntity {

   public:

      ENABLE_VTABLE();

      typedef std::vector<CCameraEntity*> TList;

   public:

      CCameraEntity(CComposableEntity* pc_parent);

      CCameraEntity(CComposableEntity* pc_parent,
                            const CRadians& c_field_of_view,
                            const CRadians& c_roll,
                            Real f_range,
                            Real f_horizontal_resolution,
                            Real f_vertical_resolution);

      virtual ~CCameraEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      //virtual void Reset() {}

      //virtual void SetEnabled(bool b_enabled) {}

      const CRadians& GetFieldOfView() const {
         return m_cFieldOfView;
      }

      Real GetRange() const {
         return m_fRange;
      }

      const CRadians& GetRoll() const {
         return m_cRoll;
      }

      const CSquareMatrix<3> GetCameraMatrix() const {
         return m_cCameraMatrix;
      }

      const CMatrix<1,5> GetDistortionParameters() const {
         return m_cDistortionParameters;
      }

      Real GetHorizontalResolution() const {
         return m_fHorizontalResolution;
      }

      Real GetVerticalResolution() const {
         return m_fVerticalResolution;
      }

      virtual std::string GetTypeDescription() const {
         return "camera";
      }

   protected:

      CRadians m_cFieldOfView;
      CRadians m_cRoll;
      Real m_fRange;
   
      CSquareMatrix<3> m_cCameraMatrix;
      CMatrix<1,5> m_cDistortionParameters;
      Real m_fHorizontalResolution;
      Real m_fVerticalResolution;

   };

   /****************************************/
   /****************************************/

}

#endif
