/**
 * @file <argos3/plugins/robots/prototype/simulator/sensors/camera_sensor_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CAMERA_SENSOR_ALGORITHM_H
#define CAMERA_SENSOR_ALGORITHM_H

namespace argos {
	class CCameraSensorAlgorithm;
}

namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    */

   // class CCameraSensorAlgorithm : public CBaseConfigurableResource
   class CCameraSensorAlgorithm {
      
   public:
      typedef std::vector<CCameraSensorAlgorithm*> TList;
      typedef std::map<std::string, CCameraSensorAlgorithm*> TMap;

   public:


      virtual void Init(TConfigurationNode& t_tree) {}

      virtual void Reset() {}

      virtual void Destroy() {}

      virtual void SetCamera(CCameraEntity& c_camera_entity) {
         m_pcCameraEntity = &c_camera_entity;   
      }

      virtual const CCI_CamerasSensor::SResult& Update() = 0;
      

      
   protected:
      
      CCameraEntity* m_pcCameraEntity;
   };   
}

#define REGISTER_CAMERA_SENSOR_ALGORITHM(CLASSNAME,             \
                                         LABEL,                 \
                                         AUTHOR,                \
                                         VERSION,               \
                                         BRIEF_DESCRIPTION,     \
                                         LONG_DESCRIPTION,      \
                                         STATUS)                \
   REGISTER_SYMBOL(CCameraSensorAlgorithm,          \
                   CLASSNAME,                       \
                   LABEL,                           \
                   AUTHOR,                          \
                   VERSION,                         \
                   BRIEF_DESCRIPTION,               \
                   LONG_DESCRIPTION,                \
                   STATUS)

#endif
