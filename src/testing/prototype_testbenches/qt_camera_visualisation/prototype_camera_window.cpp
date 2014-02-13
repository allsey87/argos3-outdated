#include "prototype_camera_window.h"

#include <QVBoxLayout>
#include <QGraphicsScene>
#include <QGraphicsView>

/********************************************************************************/
/********************************************************************************/

CPrototypeCameraWindow::CPrototypeCameraWindow(QWidget* pc_parent,
                                               CCI_PrototypeCamerasSensor* pc_camera,
                                               UInt32 un_index) :
   QDialog(pc_parent),
   m_pcCamera(pc_camera),
   m_unCameraIndex(un_index) {
   /* window configuration */
   m_pcScene = new QGraphicsScene(0.0f,
                                  0.0f,
                                  m_pcCamera->GetDescriptors()[m_unCameraIndex].HorizontalResolution,
                                  m_pcCamera->GetDescriptors()[m_unCameraIndex].VerticalResolution);
   m_pcViewport = new QGraphicsView(m_pcScene);
   m_pcLayout = new QVBoxLayout;
   m_pcLayout->addWidget(m_pcViewport);
   setLayout(m_pcLayout);
   setWindowTitle(m_pcCamera->GetDescriptors()[m_unCameraIndex].Id.c_str());
   /* Draw sensor readings */
   Update();
   /* Show the dialog */
   show();
}


/********************************************************************************/
/********************************************************************************/

CPrototypeCameraWindow::~CPrototypeCameraWindow() {
   delete m_pcLayout;
   delete m_pcViewport;
   delete m_pcScene;
}

/********************************************************************************/
/********************************************************************************/

void CPrototypeCameraWindow::Update() {
   m_pcScene->clear();
   m_pcViewport->setBackgroundBrush(QBrush(Qt::black, Qt::SolidPattern));

   const CCI_PrototypeCamerasSensor::SReading& sReading = 
      m_pcCamera->GetReadings()[m_unCameraIndex];

   for(UInt32 i = 0; i < sReading.ObservationList.size(); ++i) {
      QPen cPen(QColor(sReading.ObservationList[i].Color.GetRed(),
                       sReading.ObservationList[i].Color.GetGreen(),
                       sReading.ObservationList[i].Color.GetBlue()));

      m_pcScene->addEllipse(sReading.ObservationList[i].HorizontalIndex,
                            sReading.ObservationList[i].VerticalIndex,
                            5.0f,
                            5.0f,
                            cPen);
   }
}


/********************************************************************************/
/********************************************************************************/
