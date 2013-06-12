/**
 * @file <simulator/visualizations/models/qtopengl_robot.h>
 *
 * @author Michael Allwright - <michael.allwright@upb.de>
 */

#include "qtopengl_robot.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/robots/robot/simulator/robot_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real LED_RADIUS     = 0.01f;
   const GLfloat BODY_COLOR[]       = { 0.4f, 0.4f, 0.4f, 1.0f };
   const GLfloat SPECULAR[]         = { 0.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat SHININESS[]        = { 0.0f                   };
   const GLfloat EMISSION[]         = { 0.0f, 0.0f, 0.0f, 1.0f };

   /****************************************/
   /****************************************/

   CQTOpenGLRobot::CQTOpenGLRobot() :
      m_unVertices(20) {
      
      /* Reserve the needed display lists */
      m_unBaseList = glGenLists(4);
      /* References to the display lists */
      m_unBoxList      = m_unBaseList;
      m_unCylinderList = m_unBaseList + 1;
      m_unSphereList   = m_unBaseList + 2;
      m_unLEDList      = m_unBaseList + 3;
      
      /* Make box list */
      glNewList(m_unBoxList, GL_COMPILE);
      MakeBox();
      glEndList();

      /* Make cylinder list */
      glNewList(m_unCylinderList, GL_COMPILE);
      MakeCylinder();
      glEndList();

      /* Make sphere list */
      glNewList(m_unSphereList, GL_COMPILE);
      MakeSphere();
      glEndList();
       
      /* Make LED list */
      glNewList(m_unLEDList, GL_COMPILE);
      //MakeLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLRobot::~CQTOpenGLRobot() {
      glDeleteLists(m_unBaseList, 4);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLRobot::DrawBodies(CRobotEntity& c_entity) {
      
      /* Draw the bodies */
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, BODY_COLOR);
      
      for(CBodyEntity::TList::iterator itBody = c_entity.GetBodyEquippedEntity().GetAllBodies().begin();
          itBody != c_entity.GetBodyEquippedEntity().GetAllBodies().end();
          ++itBody) {
         /* Get the position of the body */
         const CVector3& cPosition = (*itBody)->GetPositionalEntity().GetPosition();
         /* Get the orientation of the body */
         const CQuaternion& cOrientation = (*itBody)->GetPositionalEntity().GetOrientation();
         CRadians cZAngle, cYAngle, cXAngle;
         cOrientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
         glPushMatrix();
         /* First, translate the body */
         glTranslatef(cPosition.GetX(), cPosition.GetY(), cPosition.GetZ());
         /* Second, rotate the body */
         glRotatef(ToDegrees(cXAngle).GetValue(), 1.0f, 0.0f, 0.0f);
         glRotatef(ToDegrees(cYAngle).GetValue(), 0.0f, 1.0f, 0.0f);
         glRotatef(ToDegrees(cZAngle).GetValue(), 0.0f, 0.0f, 1.0f);
         /* Third, scale the body */
         glScalef((*itBody)->GetGeometry().GetExtents().GetX(),
                  (*itBody)->GetGeometry().GetExtents().GetY(),
                  (*itBody)->GetGeometry().GetExtents().GetZ());
         /* Forth, draw the body by calling the correct list */
         switch((*itBody)->GetGeometry().GetTag()) {
         case CGeometry3::BOX:
            glCallList(m_unBoxList);
            break;
         case CGeometry3::CYLINDER:
            glCallList(m_unCylinderList);
            break;
         case CGeometry3::SPHERE:
            glCallList(m_unSphereList);
            break;
         }
         glPopMatrix();
      }
   }

   /****************************************/
   /****************************************/
   
   /* define a unit cube that can be stetched into prism representing the bodies */
   void CQTOpenGLRobot::MakeBox() {
      glEnable(GL_NORMALIZE);
      
      /* Set the material */
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, SHININESS);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, EMISSION);
      
      /* Let's start the actual shape */
      
      /* This part covers the top and bottom faces (parallel to XY) */
      glBegin(GL_QUADS);
      /* Bottom face */
      glNormal3f(0.0f, 0.0f, -1.0f);
      glVertex3f( 0.5f,  0.5f, 0.0f);
      glVertex3f( 0.5f, -0.5f, 0.0f);
      glVertex3f(-0.5f, -0.5f, 0.0f);
      glVertex3f(-0.5f,  0.5f, 0.0f);
      /* Top face */
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-0.5f, -0.5f, 1.0f);
      glVertex3f( 0.5f, -0.5f, 1.0f);
      glVertex3f( 0.5f,  0.5f, 1.0f);
      glVertex3f(-0.5f,  0.5f, 1.0f);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUADS);
      /* South face */
      glNormal3f(0.0f, -1.0f, 0.0f);
      glVertex3f(-0.5f, -0.5f, 1.0f);
      glVertex3f(-0.5f, -0.5f, 0.0f);
      glVertex3f( 0.5f, -0.5f, 0.0f);
      glVertex3f( 0.5f, -0.5f, 1.0f);
      /* East face */
      glNormal3f(1.0f, 0.0f, 0.0f);
      glVertex3f( 0.5f, -0.5f, 1.0f);
      glVertex3f( 0.5f, -0.5f, 0.0f);
      glVertex3f( 0.5f,  0.5f, 0.0f);
      glVertex3f( 0.5f,  0.5f, 1.0f);
      /* North face */
      glNormal3f(0.0f, 1.0f, 0.0f);
      glVertex3f( 0.5f,  0.5f, 1.0f);
      glVertex3f( 0.5f,  0.5f, 0.0f);
      glVertex3f(-0.5f,  0.5f, 0.0f);
      glVertex3f(-0.5f,  0.5f, 1.0f);
      /* West face */
      glNormal3f(-1.0f, 0.0f, 0.0f);
      glVertex3f(-0.5f,  0.5f, 1.0f);
      glVertex3f(-0.5f,  0.5f, 0.0f);
      glVertex3f(-0.5f, -0.5f, 0.0f);
      glVertex3f(-0.5f, -0.5f, 1.0f);
      glEnd();
      /* The shape definitions is finished */
      
      /* We don't need it anymore */
      glDisable(GL_NORMALIZE);
   }
   
   /****************************************/
   /****************************************/

   void CQTOpenGLRobot::MakeCylinder() {
      /* Since this shape can be stretched,
         make sure the normal vectors are unit-long */
      glEnable(GL_NORMALIZE);

      /* Set the material */
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, SPECULAR);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, SHININESS);
      glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, EMISSION);

      /* Let's start the actual shape */
      /* Side surface */
      CVector2 cVertex(0.5f, 0.0f);
      CRadians cAngle(CRadians::TWO_PI / m_unVertices);
      glBegin(GL_QUAD_STRIP);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glNormal3f(cVertex.GetX(), cVertex.GetY(), 0.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), 1.0f);
         glVertex3f(cVertex.GetX(), cVertex.GetY(), 0.0f);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Top disk */
      cVertex.Set(0.5f, 0.0f);
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, 1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), 1.0f);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* Bottom disk */
      cVertex.Set(0.5f, 0.0f);
      cAngle = -cAngle;
      glBegin(GL_POLYGON);
      glNormal3f(0.0f, 0.0f, -1.0f);
      for(GLuint i = 0; i <= m_unVertices; i++) {
         glVertex3f(cVertex.GetX(), cVertex.GetY(), 0.0f);
         cVertex.Rotate(cAngle);
      }
      glEnd();
      /* The shape definition is finished */

      /* We don't need it anymore */
      glDisable(GL_NORMALIZE);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLRobot::MakeSphere() {
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

   class CQTOpenGLOperationDrawRobotNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CRobotEntity& c_entity) {
         static CQTOpenGLRobot m_cModel;
         if(c_entity.HasControllableEntity()) {
            c_visualization.DrawRays(c_entity.GetControllableEntity());
         }
         m_cModel.DrawBodies(c_entity);
      }
   };

   class CQTOpenGLOperationDrawRobotSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CRobotEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLWidget, CQTOpenGLOperationDrawRobotNormal, void, CRobotEntity);

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLWidget, CQTOpenGLOperationDrawRobotSelected, void, CRobotEntity);

   /****************************************/
   /****************************************/

}
