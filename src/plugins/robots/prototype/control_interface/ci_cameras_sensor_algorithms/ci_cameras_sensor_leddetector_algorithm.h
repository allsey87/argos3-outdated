/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CI_CAMERAS_SENSOR_LEDDETECTOR_ALGORITHM_H
#define CI_CAMERAS_SENSOR_LEDDETECTOR_ALGORITHM_H

namespace argos {
	class CCI_CamerasSensorLEDDetectorAlgorithm;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithm.h>

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/datatypes/datatypes.h>

#ifdef ARGOS_WITH_LUA
extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}
#endif


namespace argos {
   
   class CCI_CamerasSensorLEDDetectorAlgorithm : virtual public CCI_CamerasSensorAlgorithm {
      
   public:

      struct SReading {
         /* Color */
         CColor Color;
         /* Coordinates in image */
         UInt32 HorizontalIndex;
         UInt32 VerticalIndex;
         /**
          * Constructor
          */
         SReading() :
            Color(CColor::BLACK),
            HorizontalIndex(0),
            VerticalIndex(0) {}
         /**
          * Constructor with parameters
          * @param c_color Observation color
          * @param un_horizontal_index horizontal index
          * @param un_vertical_index vertical index
          */
         SReading(const CColor& c_color,
               UInt32 un_horizontal_index,
               UInt32 un_vertical_index) :
            Color(c_color),
            HorizontalIndex(un_horizontal_index),
            VerticalIndex(un_vertical_index) {}
         /**
          * Vector of readings.
          */
         typedef std::vector<SReading> TList;
      };

   public:
      
      /**
       * Constructor
       */
      CCI_CamerasSensorLEDDetectorAlgorithm() {
      }
      
      /**
       * Destructor
       */
      virtual ~CCI_CamerasSensorLEDDetectorAlgorithm() {
      }
      
      const SReading::TList& GetReadings() const {
         return m_tReadings;
      }
      
#ifdef ARGOS_WITH_LUA
      virtual void CreateLuaState(lua_State* pt_lua_state);
      
      virtual void ReadingsToLuaState(lua_State* pt_lua_state);
#endif

   protected:

      SReading::TList m_tReadings;
      
   };
   
}

#endif
