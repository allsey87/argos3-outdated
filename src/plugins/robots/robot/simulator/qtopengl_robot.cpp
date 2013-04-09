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
   const GLfloat BODY_COLOR[]       = { 0.2f, 0.7f, 0.2f, 1.0f };
   const GLfloat SPECULAR[]         = { 0.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat SHININESS[]        = { 0.0f                   };
   const GLfloat EMISSION[]         = { 0.0f, 0.0f, 0.0f, 1.0f };

   /****************************************/
   /****************************************/

   CQTOpenGLRobot::CQTOpenGLRobot() :
      m_unVertices(20) {
      
      /* Reserve the needed display lists */
      m_unBaseList = glGenLists(2);
      m_unBodyList = m_unBaseList;
      m_unLEDList = m_unBaseList + 1;
      
      /* Make body list */
      glNewList(m_unBodyList, GL_COMPILE);
      MakeBody();
      glEndList();
       
      /* Make LED list */
      glNewList(m_unLEDList, GL_COMPILE);
      //MakeLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLRobot::~CQTOpenGLRobot() {
      glDeleteLists(m_unBaseList, 2);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLRobot::DrawBodies(CRobotEntity& c_entity) {
      
      CBodyEquippedEntity& cBodyEquippedEntity = c_entity.GetBodyEquippedEntity();
      /* Draw the bodies */
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, BODY_COLOR);
      
      for(UInt32 i = 0; i < cBodyEquippedEntity.GetAllBodies().size(); ++i) {
         
         CBodyEntity& cBody = cBodyEquippedEntity.GetBody(i);
         
         /* Get the position of the body */
         const CVector3& cPosition = cBody.GetPositionalEntity().GetPosition();
         /* Get the orientation of the body */
         const CQuaternion& cOrientation = cBody.GetPositionalEntity().GetOrientation();
         CRadians cZAngle, cYAngle, cXAngle;
         cOrientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
         glPushMatrix();
            /* First, translate the body */
            glTranslatef(cPosition.GetX(), cPosition.GetY(), cPosition.GetZ());
            /* Second, rotate the body */
            glRotatef(ToDegrees(cXAngle).GetValue(), 1.0f, 0.0f, 0.0f);
            glRotatef(ToDegrees(cYAngle).GetValue(), 0.0f, 1.0f, 0.0f);
            glRotatef(ToDegrees(cZAngle).GetValue(), 0.0f, 0.0f, 1.0f);
            
            glScalef(cBody.GetSize().GetX(), cBody.GetSize().GetY(), cBody.GetSize().GetZ());
            glCallList(m_unBodyList);
         glPopMatrix();
      }
   }

   /****************************************/
   /****************************************/
   
   /* define a unit cube that can be stetched into prism representing the bodies */
   void CQTOpenGLRobot::MakeBody() {
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

   class CQTOpenGLOperationDrawRobotNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CRobotEntity& c_entity) {
         static CQTOpenGLRobot m_cModel;
         //c_visualization.DrawRays(c_entity.GetControllableEntity());
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
