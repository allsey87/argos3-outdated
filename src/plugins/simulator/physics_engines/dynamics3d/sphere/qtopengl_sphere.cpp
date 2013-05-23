/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/qtopengl_sphere.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "qtopengl_sphere.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/sphere_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real LED_RADIUS = 0.01f;
   const GLfloat MOVABLE_COLOR[]    = { 0.0f, 1.0f, 0.0f, 1.0f };
   const GLfloat NONMOVABLE_COLOR[] = { 0.7f, 0.7f, 0.7f, 1.0f };
   const GLfloat SPECULAR[]         = { 0.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat SHININESS[]        = { 0.0f                   };
   const GLfloat EMISSION[]         = { 0.0f, 0.0f, 0.0f, 1.0f };

   /****************************************/
   /****************************************/

   CQTOpenGLSphere::CQTOpenGLSphere() :
      m_unVertices(20) {

      /* Reserve the needed display lists */
      m_unBaseList = glGenLists(1);
      m_unBodyList = m_unBaseList;
      m_unLEDList = m_unBaseList + 1;

      /* Make body list */
      glNewList(m_unBodyList, GL_COMPILE);
      MakeBody();
      glEndList();

      /* Make LED list */
      glNewList(m_unLEDList, GL_COMPILE);
      MakeLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLSphere::~CQTOpenGLSphere() {
      glDeleteLists(m_unBaseList, 2);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLSphere::DrawLEDs(CSphereEntity& c_entity) {
      /* Draw the LEDs */
      GLfloat pfColor[]           = {   0.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfSpecular[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      const GLfloat pfShininess[] = { 100.0f                   };
      const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
      CLEDEquippedEntity& cLEDEquippedEntity = c_entity.GetLEDEquippedEntity();
      for(UInt32 i = 0; i < cLEDEquippedEntity.GetAllLEDs().size(); ++i) {
         glPushMatrix();
         /* Set the material */
         const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
         pfColor[0] = cColor.GetRed();
         pfColor[1] = cColor.GetGreen();
         pfColor[2] = cColor.GetBlue();
         glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
         /* Perform rototranslation */
         const CVector3& cPosition = cLEDEquippedEntity.GetLED(i).GetPosition();
         glTranslatef(cPosition.GetX(), cPosition.GetY(), cPosition.GetZ());
         /* Draw the LED */
         glCallList(m_unLEDList);
         glPopMatrix();
      }
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLSphere::Draw(CSphereEntity& c_entity) {
      /* Draw the body */
      if(c_entity.GetEmbodiedEntity().IsMovable()) {
         glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, MOVABLE_COLOR);
      }
      else {
         glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, NONMOVABLE_COLOR);
      }
      glPushMatrix();
      glScalef(c_entity.GetRadius() * 2.0f, c_entity.GetRadius() * 2.0f, c_entity.GetRadius() * 2.0f);
      glCallList(m_unBodyList);
      glPopMatrix();
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLSphere::MakeBody()  {
      glEnable(GL_NORMALIZE);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, SHININESS);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, EMISSION);

      /* Let's start the actual shape */
      CVector3 cNormal, cPoint;
      CRadians cSlice(CRadians::TWO_PI / m_unVertices);
      
      glBegin(GL_TRIANGLE_STRIP);
      for(CRadians cInclination; cInclination <= CRadians::PI; cInclination += cSlice) {
         for(CRadians cAzimuth; cAzimuth <= CRadians::TWO_PI; cAzimuth += cSlice) {

            cPoint.FromSphericalCoords(0.5f, cInclination, cAzimuth);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);

            cPoint.FromSphericalCoords(0.5f, cInclination + cSlice, cAzimuth);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);

            cPoint.FromSphericalCoords(0.5f, cInclination, cAzimuth + cSlice);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);

            cPoint.FromSphericalCoords(0.5f, cInclination + cSlice, cAzimuth + cSlice);
            glNormal3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ() + 0.5f);

         }
      }
      glEnd();

      /* We don't need it anymore */
      glDisable(GL_NORMALIZE);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLSphere::MakeLED() {
      CVector3 cNormal, cPoint;
      CRadians cSlice(CRadians::TWO_PI / m_unVertices);

      glBegin(GL_TRIANGLE_STRIP);
      for(CRadians cInclination; cInclination <= CRadians::PI; cInclination += cSlice) {
         for(CRadians cAzimuth; cAzimuth <= CRadians::TWO_PI; cAzimuth += cSlice) {

            cNormal.FromSphericalCoords(1.0f, cInclination, cAzimuth);
            cPoint = LED_RADIUS * cNormal;
            glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

            cNormal.FromSphericalCoords(1.0f, cInclination + cSlice, cAzimuth);
            cPoint = LED_RADIUS * cNormal;
            glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

            cNormal.FromSphericalCoords(1.0f, cInclination, cAzimuth + cSlice);
            cPoint = LED_RADIUS * cNormal;
            glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

            cNormal.FromSphericalCoords(1.0f, cInclination + cSlice, cAzimuth + cSlice);
            cPoint = LED_RADIUS * cNormal;
            glNormal3f(cNormal.GetX(), cNormal.GetY(), cNormal.GetZ());
            glVertex3f(cPoint.GetX(), cPoint.GetY(), cPoint.GetZ());

         }
      }
      glEnd();
   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawSphereNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CSphereEntity& c_entity) {
         static CQTOpenGLSphere m_cModel;
         c_visualization.DrawPositionalEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawSphereSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CSphereEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLWidget, CQTOpenGLOperationDrawSphereNormal, void, CSphereEntity);

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLWidget, CQTOpenGLOperationDrawSphereSelected, void, CSphereEntity);

   /****************************************/
   /****************************************/

}
