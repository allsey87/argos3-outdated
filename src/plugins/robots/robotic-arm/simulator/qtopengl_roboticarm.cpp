/**
 * @file <simulator/visualizations/models/qtopengl_roboticarm.h>
 *
 * @author Michael Allwright - <michael.allwright@upb.de>
 */

#include "qtopengl_roboticarm.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/robotic-arm/simulator/roboticarm_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos {

   /****************************************/
   /****************************************/

   CQTOpenGLRoboticArm::CQTOpenGLRoboticArm() :
      m_unVertices(20) {

      /* Reserve the needed display lists */
      m_unBodyList = glGenLists(1);

      /* Make body list */
      glNewList(m_unBodyList, GL_COMPILE);
      MakeBody();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLRoboticArm::~CQTOpenGLRoboticArm() {
      glDeleteLists(m_unBodyList, 1);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLRoboticArm::Draw(CRoboticArmEntity& c_entity) {
      /* Draw the body */
      glPushMatrix();
      glTranslatef(0.0f, 0.0f, 0.05f);
      glScalef(0.05f, 0.05f, 0.05f);
      glCallList(m_unBodyList);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLRoboticArm::MakeBody() {
      glEnable(GL_NORMALIZE);

      /* Set the material */
      const GLfloat pfColor[]     = { 1.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfShininess[] = { 0.0f                   };
      const GLfloat pfEmission[]  = { 0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);

      /* Let's start the actual shape */
      CVector3 cNormal, cPoint;
      CRadians cSlice(CRadians::TWO_PI / m_unVertices);
      
      glBegin(GL_TRIANGLE_STRIP);
      for(CRadians cInclination; cInclination <= CRadians::PI; cInclination += cSlice) {
         for(CRadians cAzimuth; cAzimuth <= CRadians::TWO_PI; cAzimuth += cSlice) {

            cPoint.FromSphericalCoords(1.0f, cInclination, cAzimuth);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

            cPoint.FromSphericalCoords(1.0f, cInclination + cSlice, cAzimuth);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

            cPoint.FromSphericalCoords(1.0f, cInclination, cAzimuth + cSlice);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

            cPoint.FromSphericalCoords(1.0f, cInclination + cSlice, cAzimuth + cSlice);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

         }
      }
      glEnd();
      
      /* We don't need it anymore */
      glDisable(GL_NORMALIZE);

   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawRoboticArmNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CRoboticArmEntity& c_entity) {
         static CQTOpenGLRoboticArm m_cModel;
         //c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawPositionalEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawRoboticArmSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CRoboticArmEntity& c_entity) {
         static CQTOpenGLRoboticArm m_cModel;
         glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
         c_visualization.DrawPositionalEntity(c_entity.GetEmbodiedEntity());
         glScalef(1.2, 1.2, 1.2);
         m_cModel.Draw(c_entity);
         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }
   };

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLWidget, CQTOpenGLOperationDrawRoboticArmNormal, void, CRoboticArmEntity);

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLWidget, CQTOpenGLOperationDrawRoboticArmSelected, void, CRoboticArmEntity);

   /****************************************/
   /****************************************/

}
