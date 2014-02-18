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

#include <argos3/core/utility/plugins/factory.h>
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>

namespace argos {

   class CCamerasSensorSimulatedAlgorithm {
      
   public:
      typedef std::vector<CCamerasSensorSimulatedAlgorithm*> TList;
      typedef std::map<std::string, CCamerasSensorSimulatedAlgorithm*, std::less<std::string> > TMap;

   public:

      struct SViewport {
         CVector3 CameraLocation;
         CVector3 Position;
         CQuaternion Orientation;
         CVector3 HalfExtents;
      };

   public:

      virtual void SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index) = 0;

      virtual void SetViewport(const SViewport& s_viewport) = 0;

      virtual void Update() = 0;

      const std::vector<std::pair<bool, CRay3> >& GetCheckedRays() const {
         return m_vecCheckedRays;
      }

   protected:
      
      std::vector<std::pair<bool, CRay3> > m_vecCheckedRays;

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
