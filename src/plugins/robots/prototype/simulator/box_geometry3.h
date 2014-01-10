/**
 * @file <argos3/plugins/robots/prototype/simulator/box_geometry3.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef BOX_GEOMETRY3_H
#define BOX_GEOMETRY3_H

#include <argos3/plugins/robots/prototype/simulator/geometry3.h>

namespace argos {

   class CBoxGeometry3 : public CGeometry3 {
      public:
         CBoxGeometry3(const CVector3& c_size) :
            m_cSize(c_size) {}

         EGeometry3Tag GetTag() const {
            return BOX;
         }
      
         CVector3 GetExtents() const {
            return m_cSize;
         }

      private:
         CVector3 m_cSize;
   };
}

#endif
