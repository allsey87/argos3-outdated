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

      virtual void Draw(CRoboticArmEntity& c_entity);

   private:

      void MakeBody();

   private:
      GLuint m_unBodyList;
      GLuint m_unVertices;

   };

}

#endif
