/**
 * @file <simulator/visualizations/models/qtopengl_roboticarm.h>
 *
 * @author Michael Allwright - <michael.allwright@upb.de>
 */

#ifndef QTOPENGL_ROBOTIC_ARM_H
#define QTOPENGL_ROBOTIC_ARM_H

namespace argos {
   class CRoboticArmEntity;
}


#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLRoboticArm {

   public:

      CQTOpenGLRoboticArm();

      virtual ~CQTOpenGLRoboticArm();

      virtual void DrawLinks(CRoboticArmEntity& c_entity);

   private:

      void MakeLink();

   private:
      GLuint m_unBaseList;
      GLuint m_unLinkList;
      GLuint m_unLEDList;
      GLuint m_unVertices;

   };

}

#endif
