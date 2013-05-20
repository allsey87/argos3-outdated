/**
 * @file <argos3/plugins/robots/robot/simulator/cylinder_geometry3.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifdef CYLINDER_GEOMETRY3_H
#define CYLINDER_GEOMETRY3_H

#include <argos3/plugins/robots/robot/simulator/geometry3.h>

namespace argos {

   class CCylinderGeometry3 : public CGeometry3 {
      public:
         CCylinderGeometry3(Real f_radius, Real f_height) :
            m_fRadius(f_radius),
            m_fHeight(f_height) {}

         EGeometry3Tag GetTag() const {
            return CYLINDER;
         }
      
         CVector3 GetExtents() const {
            return CVector3(m_fRadius * 2.0f, m_fRadius * 2.0f, m_fHeight);
         }

         Real GetRadius() const {
            return m_fRadius;
         }

         Real GetHeight() const {
            return m_fHeight;
         }

      private:
         Real m_fRadius;
         Real m_fHeight;
   };
}

#endif
