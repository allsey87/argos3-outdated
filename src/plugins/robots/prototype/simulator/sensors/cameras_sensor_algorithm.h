/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_sensor_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_SENSOR_ALGORITHM_H
#define CAMERAS_SENSOR_ALGORITHM_H

namespace argos {
   class CCamerasSensorSimulatedAlgorithm;
}

#include <array>

#include <argos3/core/utility/plugins/factory.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/plane.h>
#include <argos3/core/utility/math/matrix/transformationmatrix3.h>

#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>

namespace argos {

   class CCamerasSensorSimulatedAlgorithm {
   public:
      using TVector = std::vector<CCamerasSensorSimulatedAlgorithm*>;
      using TMap = std::map<std::string, CCamerasSensorSimulatedAlgorithm*, std::less<std::string> >;

   public:
      struct SData {
         /* Camera data */
         CVector3 CameraLocation, Up, LookAt;
         /* Transformation matrices */
         CTransformationMatrix3 CameraToBodyTransform;
         CTransformationMatrix3 CameraToWorldTransform;
         CTransformationMatrix3 WorldToCameraTransform;
         /* Projection matrix */
         CSquareMatrix<3> ProjectionMatrix;
         /* Frustum bounding box */
         CVector3 BoundingBoxPosition;
         CVector3 BoundingBoxHalfExtents;
         /* Frustum planes */
         std::array<CPlane, 6> BoundingPlanes;
         /* Frustum plane data */
         Real NearDepth, NearPlaneWidth, NearPlaneHeight;
         CVector3 NearTopLeft, NearTopRight, NearBottomRight, NearBottomLeft;
         Real FarDepth, FarPlaneWidth, FarPlaneHeight;
         CVector3 FarTopLeft, FarTopRight, FarBottomRight, FarBottomLeft;
         /* Type aliases */
         using TVector = std::vector<SData>;
         using TVectorIterator = std::vector<SData>::iterator;
         using TVectorConstIterator = std::vector<SData>::const_iterator;
      };


   public:
      void SetData(const SData& s_data) {
         m_psData = &s_data;
      }

      CRadians GetAngleWithCamera(const CPositionalEntity& c_entity) {
         CVector3 cEntityToCamera(m_psData->CameraLocation - c_entity.GetPosition());
         CVector3 cEntityDirection(CVector3::Z);
         cEntityDirection.Rotate(c_entity.GetOrientation());
         Real fDotProduct = cEntityDirection.DotProduct(cEntityToCamera);
         return ACos(fDotProduct / (cEntityDirection.Length() * cEntityToCamera.Length()));
      }

      CVector2 ProjectOntoSensor(const CVector3& c_vector) {
         CVector3 cCameraToEntityTranslation(m_psData->WorldToCameraTransform * c_vector);
         /* this could be avoided if CVector3 inherited from CMatrix<3,1> */
         CMatrix<3,1> cCameraToEntityTranslationMatrix {
            cCameraToEntityTranslation.GetX() / cCameraToEntityTranslation.GetZ(),
            cCameraToEntityTranslation.GetY() / cCameraToEntityTranslation.GetZ(),
            1.0
         };
         /* get image coordinates */              
         CMatrix<3,1> cImageCoordinates(m_psData->ProjectionMatrix * cCameraToEntityTranslationMatrix);
         /* return as vector2 */
         return CVector2(cImageCoordinates(0,0), cImageCoordinates(1,0));
      }

      bool IsPointInsideFrustum(const CVector3& c_point) {
         for(const CPlane& c_plane : m_psData->BoundingPlanes) {
            if(c_plane.GetNormal().DotProduct(c_point - c_plane.GetPosition()) < 0.0) {
               return false;
            }
         }
         return true;
      }

      virtual void Update() = 0;

      const std::vector<std::pair<bool, CRay3> >& GetCheckedRays() const {
         return m_vecCheckedRays;
      }

   protected:
      std::vector<std::pair<bool, CRay3> > m_vecCheckedRays;
      const SData* m_psData;
   };   
}

#define REGISTER_CAMERAS_SENSOR_ALGORITHM(CLASSNAME,            \
                                         LABEL,                 \
                                         AUTHOR,                \
                                         VERSION,               \
                                         BRIEF_DESCRIPTION,     \
                                         LONG_DESCRIPTION,      \
                                         STATUS)                \
   REGISTER_SYMBOL(CCamerasSensorSimulatedAlgorithm,            \
                   CLASSNAME,                                   \
                   LABEL,                                       \
                   AUTHOR,                                      \
                   VERSION,                                     \
                   BRIEF_DESCRIPTION,                           \
                   LONG_DESCRIPTION,                            \
                   STATUS)

#endif
