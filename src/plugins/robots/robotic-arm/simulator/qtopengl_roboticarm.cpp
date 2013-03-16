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

   static const Real LED_RADIUS     = 0.01f;
   const GLfloat LINK_COLOR[]       = { 0.7f, 0.7f, 0.7f, 1.0f };
   const GLfloat SPECULAR[]         = { 0.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat SHININESS[]        = { 0.0f                   };
   const GLfloat EMISSION[]         = { 0.0f, 0.0f, 0.0f, 1.0f };

   /****************************************/
   /****************************************/

   CQTOpenGLRoboticArm::CQTOpenGLRoboticArm() :
      m_unVertices(20) {
      
      /* Reserve the needed display lists */
      m_unBaseList = glGenLists(2);
      m_unLinkList = m_unBaseList;
      m_unLEDList = m_unBaseList + 1;
      
      /* Make link list */
      glNewList(m_unLinkList, GL_COMPILE);
      MakeLink();
      glEndList();
       
      /* Make LED list */
      glNewList(m_unLEDList, GL_COMPILE);
      //MakeLED();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLRoboticArm::~CQTOpenGLRoboticArm() {
      glDeleteLists(m_unBaseList, 2);
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLRoboticArm::DrawLinks(CRoboticArmEntity& c_entity) {
      
      CLinkEquippedEntity& cLinkEquippedEntity = c_entity.GetLinkEquippedEntity();
      /* Draw the links */
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, LINK_COLOR);
      
      for(UInt32 i = 0; i < cLinkEquippedEntity.GetAllLinks().size(); ++i) {
         
         CLinkEntity& cLink = cLinkEquippedEntity.GetLink(i);
         
         /* Get the position of the link */
         const CVector3& cPosition = cLink.GetPositionalEntity().GetPosition();
         /* Get the orientation of the link */
         const CQuaternion& cOrientation = cLink.GetPositionalEntity().GetOrientation();
         CRadians cZAngle, cYAngle, cXAngle;
         cOrientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
         glPushMatrix();
            /* First, translate the link */
            glTranslatef(cPosition.GetX(), cPosition.GetY(), cPosition.GetZ());
            /* Second, rotate the link */
            glRotatef(ToDegrees(cXAngle).GetValue(), 1.0f, 0.0f, 0.0f);
            glRotatef(ToDegrees(cYAngle).GetValue(), 0.0f, 1.0f, 0.0f);
            glRotatef(ToDegrees(cZAngle).GetValue(), 0.0f, 0.0f, 1.0f);
            
            glScalef(0.025, 0.025, 0.2);
            glCallList(m_unLinkList);
         glPopMatrix();
      }
   }

   /****************************************/
   /****************************************/
   
   /* define a unit cube that can be stetched into prism representing the links */
   void CQTOpenGLRoboticArm::MakeLink() {
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
      glVertex3f( 0.5f,  0.5f, -0.5f);
      glVertex3f( 0.5f, -0.5f, -0.5f);
      glVertex3f(-0.5f, -0.5f, -0.5f);
      glVertex3f(-0.5f,  0.5f, -0.5f);
      /* Top face */
      glNormal3f(0.0f, 0.0f, 1.0f);
      glVertex3f(-0.5f, -0.5f, 0.5f);
      glVertex3f( 0.5f, -0.5f, 0.5f);
      glVertex3f( 0.5f,  0.5f, 0.5f);
      glVertex3f(-0.5f,  0.5f, 0.5f);
      glEnd();
      /* This part covers the faces (South, East, North, West) */
      glBegin(GL_QUADS);
         /* South face */
         glNormal3f(0.0f, -1.0f, 0.0f);
         glVertex3f(-0.5f, -0.5f,  0.5f);
         glVertex3f(-0.5f, -0.5f, -0.5f);
         glVertex3f( 0.5f, -0.5f, -0.5f);
         glVertex3f( 0.5f, -0.5f,  0.5f);
         /* East face */
         glNormal3f(1.0f, 0.0f, 0.0f);
         glVertex3f( 0.5f, -0.5f,  0.5f);
         glVertex3f( 0.5f, -0.5f, -0.5f);
         glVertex3f( 0.5f,  0.5f, -0.5f);
         glVertex3f( 0.5f,  0.5f,  0.5f);
         /* North face */
         glNormal3f(0.0f, 1.0f, 0.0f);
         glVertex3f( 0.5f,  0.5f,  0.5f);
         glVertex3f( 0.5f,  0.5f, -0.5f);
         glVertex3f(-0.5f,  0.5f, -0.5f);
         glVertex3f(-0.5f,  0.5f,  0.5f);
         /* West face */
         glNormal3f(-1.0f, 0.0f, 0.0f);
         glVertex3f(-0.5f,  0.5f,  0.5f);
         glVertex3f(-0.5f,  0.5f, -0.5f);
         glVertex3f(-0.5f, -0.5f, -0.5f);
         glVertex3f(-0.5f, -0.5f,  0.5f);
      glEnd();
      /* The shape definitions is finished */
      
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
         m_cModel.DrawLinks(c_entity);
      }
   };

   class CQTOpenGLOperationDrawRoboticArmSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CRoboticArmEntity& c_entity) {
         static CQTOpenGLRoboticArm m_cModel;
         glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
         glScalef(1.2, 1.2, 1.2);
         m_cModel.DrawLinks(c_entity);
         glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }
   };

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLWidget, CQTOpenGLOperationDrawRoboticArmNormal, void, CRoboticArmEntity);

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLWidget, CQTOpenGLOperationDrawRoboticArmSelected, void, CRoboticArmEntity);

   /****************************************/
   /****************************************/

}
