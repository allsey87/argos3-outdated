/**
 * @file <argos3/plugins/robots/prototype/simulator/qtopengl_prototype.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef QTOPENGL_PROTOTYPE_H
#define QTOPENGL_PROTOTYPE_H

namespace argos {
   class CPrototypeEntity;
}


#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

namespace argos {

   class CQTOpenGLPrototype {

   public:

      CQTOpenGLPrototype();

      virtual ~CQTOpenGLPrototype();

      virtual void DrawBodies(CPrototypeEntity& c_entity);
      virtual void DrawDevices(CPrototypeEntity& c_entity);

   private:

      void MakeBox();
      void MakeCylinder();
      void MakeSphere();
      void MakeLED();

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
