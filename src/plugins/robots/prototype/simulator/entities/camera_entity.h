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
                    const CRange<Real>& c_range,
                    const CSquareMatrix<3>& c_camera_matrix,
                    const CMatrix<1,5>& c_distortion_parameters,
                    Real f_horizontal_resolution,
                    Real f_vertical_resolution);

      virtual ~CCameraEntity() {}

      virtual void Init(TConfigurationNode& t_tree);

      const CRange<Real> GetRange() const {
         return m_cRange;
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
      CRange<Real> m_cRange;
      CSquareMatrix<3> m_cCameraMatrix;
      CMatrix<1,5> m_cDistortionParameters;
      Real m_fHorizontalResolution;
      Real m_fVerticalResolution;
   };

   /****************************************/
   /****************************************/

}

#endif
