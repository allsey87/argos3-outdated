/**
 * @file <argos3/plugins/robots/prototype/simulator/visualizations/qtopengl_prototype.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "qtopengl_prototype.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/prototype_led_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/electromagnet_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/tag_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/entities/camera_equipped_entity.h>
#include <argos3/plugins/robots/prototype/simulator/sensors/cameras_default_sensor.h>

namespace argos {

   /****************************************/
   /****************************************/

   static const Real LED_RADIUS     = 0.0025f;
   const GLfloat BODY_COLOR[]       = { 0.4f, 0.4f, 0.4f, 1.0f };
   const GLfloat SPECULAR[]         = { 0.0f, 0.0f, 0.0f, 1.0f };
   const GLfloat SHININESS[]        = { 0.0f                   };
   const GLfloat EMISSION[]         = { 0.0f, 0.0f, 0.0f, 1.0f };

   /****************************************/
   /****************************************/

   CQTOpenGLPrototype::CQTOpenGLPrototype() :
      m_unVertices(20) {
      
      /* Reserve the needed display lists */
      m_unBaseList = glGenLists(6);
      /* References to the display lists */
      m_unBoxList      = m_unBaseList;
      m_unCylinderList = m_unBaseList + 1;
      m_unSphereList   = m_unBaseList + 2;
      m_unLEDList      = m_unBaseList + 3;
      m_unPoleList     = m_unBaseList + 4;
      m_unTagList  = m_unBaseList + 5;
      
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
      MakeLED();
      glEndList();

      /* Make Poles list */
      glNewList(m_unPoleList, GL_COMPILE);
      MakePoles();
      glEndList();

      /* Make Tag list */
      MakeTagTexture();
      glNewList(m_unTagList, GL_COMPILE);
      MakeTag();
      glEndList();
   }

   /****************************************/
   /****************************************/

   CQTOpenGLPrototype::~CQTOpenGLPrototype() {
      glDeleteLists(m_unBaseList, 5);
   }
   
   /****************************************/
   /****************************************/
   
   void CQTOpenGLPrototype::MakeLED() {
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

   void CQTOpenGLPrototype::DrawBodies(CPrototypeEntity& c_entity) {
      /* Draw the bodies */
      for(CBodyEntity::TList::iterator itBody = c_entity.GetBodyEquippedEntity().GetAllBodies().begin();
          itBody != c_entity.GetBodyEquippedEntity().GetAllBodies().end();
          ++itBody) {
         /* Configure the body material */
         glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, BODY_COLOR);
         //glPolygonMode(GL_FRONT, GL_LINE);
         //glPolygonMode(GL_BACK, GL_LINE);

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
         //glPolygonMode(GL_FRONT, GL_FILL);
         //glPolygonMode(GL_BACK, GL_FILL);
         glPopMatrix();
      }
   }

   void CQTOpenGLPrototype::DrawDevices(CPrototypeEntity& c_entity) {
      if(c_entity.HasComponent("leds")) {
         CPrototypeLEDEquippedEntity& cLEDEquippedEntity = c_entity.GetComponent<CPrototypeLEDEquippedEntity>("leds");            
         for(UInt32 i = 0; i < cLEDEquippedEntity.GetAllLEDs().size(); ++i) {
            glPushMatrix();
            GLfloat pfColor[]           = {   0.0f, 0.0f, 0.0f, 1.0f };
            const GLfloat pfSpecular[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
            const GLfloat pfShininess[] = { 100.0f                   };
            const GLfloat pfEmission[]  = {   0.0f, 0.0f, 0.0f, 1.0f };
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, pfSpecular);
            glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, pfShininess);
            glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, pfEmission);
            /* Set the material */
            const CColor& cColor = cLEDEquippedEntity.GetLED(i).GetColor();
            pfColor[0] = cColor.GetRed() / 255.0f;
            pfColor[1] = cColor.GetGreen() / 255.0f;
            pfColor[2] = cColor.GetBlue() / 255.0f;
            glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, pfColor);
            /* Get the orientation of the LED */
            const CVector3& cPosition = cLEDEquippedEntity.GetLED(i).GetPosition();
            glTranslatef(cPosition.GetX(), cPosition.GetY(), cPosition.GetZ() - LED_RADIUS/2);
            /* Draw the LED */
            glScalef(LED_RADIUS,LED_RADIUS,LED_RADIUS);
            glCallList(m_unSphereList);
            glPopMatrix();
         }
      }
      
      /*
      if(c_entity.HasComponent("electromagnets")) {
         CElectromagnetEquippedEntity& cElectromagnetEquippedEntity = c_entity.GetComponent<CElectromagnetEquippedEntity>("electromagnets");
         for(UInt32 i = 0; i < cElectromagnetEquippedEntity.GetAllElectromagneticBodies().size(); ++i) {

            const CBodyEntity& cBody = cElectromagnetEquippedEntity.GetElectromagneticBody(i);          
            const CVector3& cBodyPosition = cBody.GetPositionalEntity().GetPosition();
            // Get the orientation of the body
            const CQuaternion& cBodyOrientation = cBody.GetPositionalEntity().GetOrientation();
            CRadians cBodyZAngle, cBodyYAngle, cBodyXAngle;                 
            cBodyOrientation.ToEulerAngles(cBodyZAngle, cBodyYAngle, cBodyXAngle);       
            const CVector3& cBodyField = cElectromagnetEquippedEntity.GetElectromagnet(i).GetField();     
            CQuaternion cFieldOrientation(CVector3::Z, cBodyField);              
            CRadians cFieldZAngle, cFieldYAngle, cFieldXAngle;                             
            cFieldOrientation.ToEulerAngles(cFieldZAngle, cFieldYAngle, cFieldXAngle);

            glPushMatrix();
            glTranslatef(cBodyPosition.GetX(), cBodyPosition.GetY(),cBodyPosition.GetZ());    
            glRotatef(ToDegrees(cBodyXAngle).GetValue(), 1.0f, 0.0f, 0.0f);
            glRotatef(ToDegrees(cBodyYAngle).GetValue(), 0.0f, 1.0f, 0.0f);
            glRotatef(ToDegrees(cBodyZAngle).GetValue(), 0.0f, 0.0f, 1.0f);
            glTranslatef(0,0, cBody.GetGeometry().GetExtents().GetZ()/2);               
            glRotatef(ToDegrees(cFieldXAngle).GetValue(), 1.0f, 0.0f, 0.0f);
            glRotatef(ToDegrees(cFieldYAngle).GetValue(), 0.0f, 1.0f, 0.0f); 
            glRotatef(ToDegrees(cFieldZAngle).GetValue(), 0.0f, 0.0f, 1.0f);
            glScalef(Abs(cBodyField.GetX() * 0.01f) + 1.0f,
                     Abs(cBodyField.GetY() * 0.01f) + 1.0f,
                     Abs(cBodyField.GetZ() * 0.01f) + 1.0f);
            glScalef(cBody.GetGeometry().GetExtents().GetX(),
                     cBody.GetGeometry().GetExtents().GetY(),
                     cBody.GetGeometry().GetExtents().GetZ());                             
            glCallList(m_unPoleList);
            glPopMatrix();                              
         }
      }
      */
      
      /* Camera Testing Begin */
      /*
      if(c_entity.HasComponent("controller")) {
         CControllableEntity& cController = c_entity.GetComponent<CControllableEntity>("controller");
         if(cController.GetController().HasSensor("cameras")) {
            CCamerasDefaultSensor* pcCameraSensor = cController.GetController().GetSensor<CCamerasDefaultSensor>("cameras");
            if(pcCameraSensor != NULL) {
               for(size_t i = 0; i < pcCameraSensor->GetViewports().size(); ++i) {
                  glPushMatrix();
                  glPolygonMode(GL_FRONT, GL_LINE);
                  glPolygonMode(GL_BACK, GL_LINE);
                  glTranslatef(pcCameraSensor->GetViewports()[i].Position.GetX(),
                               pcCameraSensor->GetViewports()[i].Position.GetY(),
                               pcCameraSensor->GetViewports()[i].Position.GetZ() -
                               pcCameraSensor->GetViewports()[i].HalfExtents[0]);
                  glScalef(pcCameraSensor->GetViewports()[i].HalfExtents[0] * 2.0f,
                           pcCameraSensor->GetViewports()[i].HalfExtents[0] * 2.0f,
                           pcCameraSensor->GetViewports()[i].HalfExtents[0] * 2.0f);
                  glCallList(m_unSphereList);
                  glPolygonMode(GL_FRONT, GL_FILL);
                  glPolygonMode(GL_BACK, GL_FILL);
                  glPopMatrix();
               }
            }
         }
      }
      */
      /* Camera Testing End */

      if(c_entity.HasComponent("tag_container")) {
         CTagEquippedEntity& cTagEquippedEntity =
            c_entity.GetComponent<CTagEquippedEntity>("tag_container");
         for(UInt32 i = 0; i < cTagEquippedEntity.GetAllTags().size(); ++i) {
            const CVector3& cTagPosition = cTagEquippedEntity.GetTag(i).GetPosition();
            const CQuaternion& cTagOrientation = cTagEquippedEntity.GetTag(i).GetOrientation(); 
            CRadians cZ, cY, cX;
            cTagOrientation.ToEulerAngles(cZ, cY, cX);
            Real fScaling = cTagEquippedEntity.GetTag(i).GetSideLength();

            glPushMatrix();
            
            glTranslatef(cTagPosition.GetX(),
                         cTagPosition.GetY(),
                         cTagPosition.GetZ());
            glRotatef(ToDegrees(cX).GetValue(), 1.0f, 0.0f, 0.0f);
            glRotatef(ToDegrees(cY).GetValue(), 0.0f, 1.0f, 0.0f); 
            glRotatef(ToDegrees(cZ).GetValue(), 0.0f, 0.0f, 1.0f);
            glScalef(fScaling, fScaling, 1.0f);
            glCallList(m_unTagList);
            glPopMatrix();
         }
      }
   }
   
   /****************************************/
   /****************************************/
   
   /* define a unit cube that can be stetched into prism representing the bodies */
   void CQTOpenGLPrototype::MakeBox() {
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

   void CQTOpenGLPrototype::MakeCylinder() {
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

   void CQTOpenGLPrototype::MakeSphere() {
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


   void CQTOpenGLPrototype::MakeTagTexture() {
      glGenTextures(1, &m_unTagTex);
      glBindTexture(GL_TEXTURE_2D, m_unTagTex);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 4, 4, 0, GL_RGB, GL_FLOAT, m_fTagTexture);
   }

   void CQTOpenGLPrototype::MakeTag() {
      glEnable(GL_NORMALIZE);
      glDisable(GL_LIGHTING);
      glEnable(GL_TEXTURE_2D);
      glBindTexture(GL_TEXTURE_2D, m_unTagTex);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);    
      glBegin(GL_QUADS);
      // tag
      glNormal3f(0.0f, 0.0f, 1.0f);
      glTexCoord2f(1.0f, 1.0f); glVertex2f( 0.5f,  0.5f);
      glTexCoord2f(0.03f, 1.0f); glVertex2f(-0.5f,  0.5f);
      glTexCoord2f(0.03f, 0.03f); glVertex2f(-0.5f, -0.5f);
      glTexCoord2f(1.0f, 0.03f); glVertex2f( 0.5f, -0.5f);
      
      glEnd();
      glDisable(GL_TEXTURE_2D);     
      glEnable(GL_LIGHTING);
      glDisable(GL_NORMALIZE);
   }


   void CQTOpenGLPrototype::MakePoles() {
      glEnable(GL_NORMALIZE);	
      glDisable(GL_LIGHTING);
      glLineWidth(4.0f);
      glBegin(GL_LINES);
      // south pole
      glColor3f(1.0, 0.0, 0.0);
      glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(0.0f, 0.0f, 1.0f);
      // north pole
      glColor3f(0.0, 0.0, 1.0);	
      glVertex3f(0.0f, 0.0f, 0.0f);
      glVertex3f(0.0f, 0.0f, -1.0f);
      glEnd();
      glLineWidth(1.0f);
      glEnable(GL_LIGHTING);
      glDisable(GL_NORMALIZE);
   }
   
   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawPrototypeNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CPrototypeEntity& c_entity) {
         static CQTOpenGLPrototype m_cModel;
         if(c_entity.HasControllableEntity()) {
            c_visualization.DrawRays(c_entity.GetControllableEntity());
         }
         m_cModel.DrawBodies(c_entity);
         m_cModel.DrawDevices(c_entity);
      }
   };

   class CQTOpenGLOperationDrawPrototypeSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CPrototypeEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLWidget, CQTOpenGLOperationDrawPrototypeNormal, void, CPrototypeEntity);

   REGISTER_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLWidget, CQTOpenGLOperationDrawPrototypeSelected, void, CPrototypeEntity);

   /****************************************/
   /****************************************/

}
