/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_prototype_forwards_camera_sensor.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CI_PROTOTYPE_FORWARDS_CAMERA_SENSOR_H
#define CI_PROTOTYPE_FORWARDS_CAMERA_SENSOR_H

namespace argos {
	class CCI_PrototypeForwardsCameraSensor;
}

#include <argos3/core/control_interface/ci_sensor.h>
#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/datatypes/color.h>
#include <vector>

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
   class CCI_PrototypeForwardsCameraSensor: virtual public CCI_Sensor {
      
   public:
      
      /**
       * An SBlob represents a generic colored 2D segment in the image.
       *
       * A blob has a color as defined by the CColor class. The position of the blob is given in
       * polar coordinates. The angle is counted counter-clockwise watching the FootBot from top, as to respect the
       * general frame convention. As for the distances, both the distance in cms from the robot center and in pixels from the
       * optical center in the image are given. The last attribute is the area of the blob in pixels.
       *
       * @see CColor
       */
      struct SObservation {
         /* Color */
         CColor Color;
         /* Coordinates in image */
         UInt32 HorizontalIndex;
         UInt32 VerticalIndex;
         /**
          * Constructor
          */
         SObservation() :
            Color(CColor::BLACK),
            HorizontalIndex(0),
            VerticalIndex(0) {}
         /**
          * Constructor with parameters
          * @param c_color Observation color
          * @param un_horizontal_index horizontal index
          * @param un_vertical_index vertical index
          */
         SObservation(const CColor& c_color,
               UInt32 un_horizontal_index,
               UInt32 un_vertical_index) :
            Color(c_color),
            HorizontalIndex(un_horizontal_index),
            VerticalIndex(un_vertical_index) {}
         /**
          * Vector of observations.
          */
         typedef std::vector<SObservation> TList;
      };


      struct SReading {
         SObservation::TList ObservationList;         
         /**
          * Vector of observation lists.
          */
         typedef std::vector<SReading> TList;
      };

      
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
      CCI_PrototypeForwardsCameraSensor() {
      }
      
      /**
       * Destructor
       */
      virtual ~CCI_PrototypeForwardsCameraSensor() {
      }
      
      /**
       * Returns a reference to the current camera readings.
       * @return A reference to the current camera readings.
       */
      inline const SReading::TList& GetReadings() const {
         return m_tReadings;
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
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:
 
      SReading::TList m_tReadings;
      SDescriptor::TList m_tDescriptors;
      
      
   };
   
}

#endif
