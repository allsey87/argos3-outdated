/**
 * @file <argos3/plugins/robots/prototype/simulator/sphere_geometry3.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef SPHERE_GEOMETRY3_H
#define SPHERE_GEOMETRY3_H

#include <argos3/plugins/robots/prototype/utility/geometry3.h>

namespace argos {

   class CSphereGeometry3 : public CGeometry3 {
      public:
         CSphereGeometry3(Real f_radius) :
            m_fRadius(f_radius) {}

         EGeometry3Tag GetTag() const {
            return SPHERE;
         }
      
         CVector3 GetExtents() const {
            return CVector3(m_fRadius * 2.0f, m_fRadius * 2.0f, m_fRadius * 2.0f);
         }

         Real GetRadius() const {
            return m_fRadius;
         }

      private:
         Real m_fRadius;
   };
}

#endif
