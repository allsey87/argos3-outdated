#include "camera_window.h"

#include <QVBoxLayout>
#include <QGraphicsScene>
#include <QGraphicsView>

/********************************************************************************/
/********************************************************************************/

CCameraWindow::CCameraWindow(QWidget* pc_parent,
                             CCI_CamerasSensor* pc_camera,
                             UInt32 un_index) :
   QDialog(pc_parent),
   m_pcCamera(pc_camera),
   m_unCameraIndex(un_index),
   m_strCameraId(m_pcCamera->GetDescriptors()[m_unCameraIndex].Id),
   m_pcLEDDetectorAlgorithm(NULL) {
   /* window configuration */
   m_pcScene = new QGraphicsScene(0.0f,
                                  0.0f,
                                  m_pcCamera->GetDescriptors()[m_unCameraIndex].HorizontalResolution,
                                  m_pcCamera->GetDescriptors()[m_unCameraIndex].VerticalResolution);

   if(m_pcCamera->HasAlgorithm(m_strCameraId, "led_detector")) {
      m_pcLEDDetectorAlgorithm = m_pcCamera->GetAlgorithm<CCI_CamerasSensorLEDDetectorAlgorithm>(m_strCameraId, "led_detector");
   }
   if(m_pcCamera->HasAlgorithm(m_strCameraId, "tag_detector")) {
      m_pcTagDetectorAlgorithm = m_pcCamera->GetAlgorithm<CCI_CamerasSensorTagDetectorAlgorithm>(m_strCameraId, "tag_detector");
   }

                                  
   m_pcViewport = new QGraphicsView(m_pcScene);
   m_pcLayout = new QVBoxLayout;
   m_pcLayout->addWidget(m_pcViewport);
   setLayout(m_pcLayout);
   setWindowTitle(m_strCameraId.c_str());
   /* Draw sensor readings */
   Update();
   /* Show the dialog */
   show();
}


/********************************************************************************/
/********************************************************************************/

CCameraWindow::~CCameraWindow() {
   delete m_pcLayout;
   delete m_pcViewport;
   delete m_pcScene;
}

/********************************************************************************/
/********************************************************************************/

void CCameraWindow::Update() {
   m_pcScene->clear();
   m_pcViewport->setBackgroundBrush(QBrush(Qt::black, Qt::SolidPattern));

   if(m_pcLEDDetectorAlgorithm != NULL) {
      const CCI_CamerasSensorLEDDetectorAlgorithm::SReading::TList& sReadings =
         m_pcLEDDetectorAlgorithm->GetReadings();      
      for(UInt32 i = 0; i < sReadings.size(); ++i) {
         QPen cPen(QColor(sReadings[i].Color.GetRed(),
                          sReadings[i].Color.GetGreen(),
                          sReadings[i].Color.GetBlue()));
         
         m_pcScene->addEllipse(sReadings[i].Center.GetX(),
                               sReadings[i].Center.GetY(),
                               5.0f,
                               5.0f,
                               cPen);
         
      }
   }

   if(m_pcTagDetectorAlgorithm != NULL) {
      const CCI_CamerasSensorTagDetectorAlgorithm::SReading::TList& sReadings =
         m_pcTagDetectorAlgorithm->GetReadings();      
      for(UInt32 i = 0; i < sReadings.size(); ++i) {
         QPen cPen(QColor(255,255,255));        
         m_pcScene->addEllipse(sReadings[i].Center.GetX(),
                               sReadings[i].Center.GetY(),
                               5.0f,
                               5.0f,
                               cPen);
         for(UInt32 j = 0; j < sReadings[i].Corners.size(); j++) {
            m_pcScene->addLine(sReadings[i].Corners[j].GetX(),
                               sReadings[i].Corners[j].GetY(),
                               sReadings[i].Corners[(j + 1) % sReadings[i].Corners.size()].GetX(),
                               sReadings[i].Corners[(j + 1) % sReadings[i].Corners.size()].GetY(),
                               cPen);
         }
      }
   }
}


/********************************************************************************/
/********************************************************************************/
