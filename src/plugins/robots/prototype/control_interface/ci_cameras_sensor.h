/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor/ci_cameras_sensor.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CI_CAMERAS_SENSOR_H
#define CI_CAMERAS_SENSOR_H

namespace argos {
	class CCI_CamerasSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithm.h>

#include <cxxabi.h>
#include <typeinfo>


namespace argos {
   
   /**
    * This class provides the most general interface to a camera.
    * The camera sensor enables the user to extract information from the images
    * acquired by the simulated or by the physical camera.
    * This interface defines also the basic type of information that at the moment
    * it is possible to extract from image processing on the real robot, that is the position
    * of the colored LED of neighboring robots.
    * The camera can be enabled and disabled, in order to save computation time.
    */
   class CCI_CamerasSensor: virtual public CCI_Sensor {
      
   public:

      struct SDescriptor {
         std::string Id;
         UInt32 HorizontalResolution;
         UInt32 VerticalResolution;
         bool Enabled;
         /**
          * Constructor
          */
         SDescriptor() : 
            Id(""),
            HorizontalResolution(0),
            VerticalResolution(0),
            Enabled(false) {}
         /**
          * Constructor with parameters
          * @param str_id Camera Indentifier
          * @param un_horizontal_resolution Horizontal Resolution
          * @param un_vertical_resolution Vertical Resolution
          * @param b_enabled Camera Enabled
          */
         SDescriptor(std::string str_id,
                     UInt32 un_horizontal_resolution,
                     UInt32 un_vertical_resolution,
                     bool b_enabled) : 
            Id(str_id),
            HorizontalResolution(un_horizontal_resolution),
            VerticalResolution(un_vertical_resolution),
            Enabled(b_enabled) {}
         /**
          * Vector of descriptors.
          */
         typedef std::vector<SDescriptor> TList;
      };

   public:
     
      /**
       * Constructor
       */
      CCI_CamerasSensor() {
      }
      
      /**
       * Destructor
       */
      virtual ~CCI_CamerasSensor() {
      }
      
      /**
       * Returns a reference to the camera descriptors
       * @return A reference to the camera descriptors
       */
      virtual const SDescriptor::TList& GetDescriptors() const {
         return m_tDescriptors;
      }
      
      /**
       * Enables image acquisition and processing.
       */
      virtual void Enable() = 0;
      
      /**
       * Disables image acquisition and processing.
       */
      virtual void Disable() = 0;
      
   public:

      bool HasAlgorithm(const std::string& str_camera_name, const std::string& str_algorithm_type) const;

      template<typename ALGORITHM_IMPL>
      ALGORITHM_IMPL* GetAlgorithm(const std::string& str_camera_name, const std::string& str_algorithm_type) {
         std::map<std::string, CCI_CamerasSensorAlgorithm::TMap, std::less<std::string> >::const_iterator itCamera =
            m_mapCameraAlgorithms.find(str_camera_name);
         if(itCamera != m_mapCameraAlgorithms.end()) {
            CCI_CamerasSensorAlgorithm::TMap::const_iterator itAlgorithm = itCamera->second.find(str_algorithm_type);
            if(itAlgorithm != itCamera->second.end()) {
               ALGORITHM_IMPL* pcAlgorithm = dynamic_cast<ALGORITHM_IMPL*>(itAlgorithm->second);
               if(pcAlgorithm != NULL) {
                  return pcAlgorithm;
               }
               else {
                  char* pchDemangledType = abi::__cxa_demangle(typeid(ALGORITHM_IMPL).name(), NULL, NULL, NULL);
                  THROW_ARGOSEXCEPTION("Algorithm type " << str_algorithm_type << " cannot be cast to type " << pchDemangledType);
               }
            }
            else {
               THROW_ARGOSEXCEPTION("Camera " << str_camera_name << " is not running the " <<
                                    str_algorithm_type << " algorithm. Did you add it to the XML file?");
            }
         }
         else {
            THROW_ARGOSEXCEPTION("Unknown camera instance " << str_camera_name <<
                                 " requested in camera sensor. Did you add it to the XML file?");
         }
      }

#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif


   protected:
 
      SDescriptor::TList m_tDescriptors;

      std::map<std::string, CCI_CamerasSensorAlgorithm::TMap, std::less<std::string> > m_mapCameraAlgorithms;
      
      
   };
   
}

#endif
