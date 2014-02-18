/**
 * @file <argos3/plugins/robot/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_apriltags_algorithm.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef CI_CAMERAS_SENSOR_APRILTAGS_ALGORITHM_H
#define CI_CAMERAS_SENSOR_APRILTAGS_ALGORITHM_H

namespace argos {
	class CCI_CamerasSensorApriltagsAlgorithm;
}

#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithm.h>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/datatypes/datatypes.h>

#ifdef ARGOS_WITH_LUA
extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}
#endif


namespace argos {
   
   class CCI_CamerasSensorApriltagsAlgorithm : virtual public CCI_CamerasSensorAlgorithm {
      
   public:

      struct SReading {
         /* Data contained in barcode */
         std::string Payload;
         /* Coordinates in image */
         UInt32 HorizontalIndex;
         UInt32 VerticalIndex;
         /* 3D transform */
         /* Only populated if barcode is localizable */
         CVector3 Position;
         CQuaternion Orientation;
         /**
          * Constructor
          */
         SReading() :
            Payload(""),
            HorizontalIndex(0),
            VerticalIndex(0) {}
         /**
          * Constructor with parameters
          * @param c_color Observation color
          * @param un_horizontal_index horizontal index
          * @param un_vertical_index vertical index
          */
         SReading(const std::string& str_payload,
                  UInt32 un_horizontal_index,
                  UInt32 un_vertical_index,
                  const CVector3& c_position = CVector3(),
                  const CQuaternion& c_orientation = CQuaternion()) :
            Payload(str_payload),
            HorizontalIndex(un_horizontal_index),
            VerticalIndex(un_vertical_index),
            Position(c_position),
            Orientation(c_orientation) {}
         /**
          * Vector of readings.
          */
         typedef std::vector<SReading> TList;
      };

   public:
      
      /**
       * Constructor
       */
      CCI_CamerasSensorApriltagsAlgorithm() {
      }
      
      /**
       * Destructor
       */
      virtual ~CCI_CamerasSensorApriltagsAlgorithm() {
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
