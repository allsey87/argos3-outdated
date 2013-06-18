/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/qtopengl_sphere.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef QTOPENGL_SPHERE_H
#define QTOPENGL_SPHERE_H

namespace argos {
   class CQTOpenGLSphere;
   class CSphereEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLSphere {

   public:

      CQTOpenGLSphere();

      virtual ~CQTOpenGLSphere();

      void DrawLEDs(CSphereEntity& c_entity);
      virtual void Draw(CSphereEntity& c_entity);

   private:

      void MakeBody();
      void MakeLED();

   private:

      GLuint m_unBaseList;
      GLuint m_unBodyList;
      GLuint m_unLEDList;
      GLuint m_unVertices;

   };

}

#endif
