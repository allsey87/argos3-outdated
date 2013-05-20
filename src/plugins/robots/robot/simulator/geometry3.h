/**
 * @file <argos3/plugins/robots/robot/simulator/geometry3.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifdef GEOMETRY3_H
#define GEOMETRY3_H

#include <argos3/core/utility/math/vector3.h>


namespace argos {

   class CGeometry3 {
      public:
      
         enum EGeometry3Tag {
            BOX,
            CYLINDER,
            SPHERE
         };

      public:
         EGeometry3Tag GetTag() const = 0;
         CVector3 GetExtents() const = 0;
   };

}

#endif
