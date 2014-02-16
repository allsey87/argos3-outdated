/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERAS_DEFAULT_SENSOR_ALGORITHM_H
#define CAMERAS_DEFAULT_SENSOR_ALGORITHM_H

namespace argos {
   class CCamerasDefaultSensorAlgorithm;
}

#include <argos3/core/utility/plugins/factory.h>
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.h>

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithm.h>

namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    */

   // class CCamerasDefaultSensorAlgorithm : public CBaseConfigurableResource
   class CCamerasDefaultSensorAlgorithm : public CCI_CamerasSensorAlgorithm {
      
   public:
      typedef std::vector<CCamerasDefaultSensorAlgorithm*> TList;
      typedef std::map<std::string, CCamerasDefaultSensorAlgorithm*> TMap;

   public:


      virtual void Init(TConfigurationNode& t_tree) {}

      virtual void Reset() {}

      virtual void Destroy() {}

      virtual void SetCamera(CCameraEquippedEntity& c_entity, UInt32 un_index) {}

      virtual void SetViewport(const CCamerasDefaultSensor::SViewport& s_viewport) {}

      virtual void Update() = 0;

   };   
}

#define REGISTER_CAMERA_SENSOR_ALGORITHM(CLASSNAME,             \
                                         LABEL,                 \
                                         AUTHOR,                \
                                         VERSION,               \
                                         BRIEF_DESCRIPTION,     \
                                         LONG_DESCRIPTION,      \
                                         STATUS)                \
   REGISTER_SYMBOL(CCamerasDefaultSensorAlgorithm,              \
                   CLASSNAME,                                   \
                   LABEL,                                       \
                   AUTHOR,                                      \
                   VERSION,                                     \
                   BRIEF_DESCRIPTION,                           \
                   LONG_DESCRIPTION,                            \
                   STATUS)

#endif
