/**
 * @file <simulator/visualizations/models/qtopengl_robot.h>
 *
 * @author Michael Allwright - <michael.allwright@upb.de>
 */

#ifndef QTOPENGL_ROBOT_H
#define QTOPENGL_ROBOT_H

namespace argos {
   class CRobotEntity;
}


#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLRobot {

   public:

      CQTOpenGLRobot();

      virtual ~CQTOpenGLRobot();

      virtual void DrawBodies(CRobotEntity& c_entity);

   private:

      void MakeBox();
      void MakeCylinder();
      void MakeSphere();

   private:
      GLuint m_unBaseList;
      GLuint m_unBoxList;
      GLuint m_unCylinderList;
      GLuint m_unSphereList;
      GLuint m_unLEDList;
      GLuint m_unVertices;

   };
}

#endif
