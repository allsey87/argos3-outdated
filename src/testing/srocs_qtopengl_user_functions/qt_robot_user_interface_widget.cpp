#include "qt_robot_user_interface_widget.h"

#include <argos3/plugins/robots/prototype/simulator/entities/prototype_entity.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_leddetector_algorithm.h>
#include <argos3/plugins/robots/prototype/control_interface/ci_cameras_sensor_algorithms/ci_cameras_sensor_tagdetector_algorithm.h>

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_log_stream.h>

#include <QVBoxLayout>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QTextEdit>
#include <QTextStream>

namespace argos {

   /********************************************************************************/
   /********************************************************************************/

   CQtRobotUserInterfaceWidget::CQtRobotUserInterfaceWidget(QWidget* pc_parent,
                                                            CCI_Controller* pc_controller) :
      QWidget(pc_parent),
      m_pcController(pc_controller),
      m_pcCamera(nullptr),
      m_pcLEDDetectorAlgorithm(nullptr),
      m_pcTagDetectorAlgorithm(nullptr) {
      m_pcCamera = m_pcController->GetSensor<CCI_CamerasSensor>("cameras");
      if(m_pcCamera->HasAlgorithm("duovero_camera", "led_detector")) {
         m_pcLEDDetectorAlgorithm = m_pcCamera->GetAlgorithm<CCI_CamerasSensorLEDDetectorAlgorithm>("duovero_camera", "led_detector");
      }
      if(m_pcCamera->HasAlgorithm("duovero_camera", "tag_detector")) {
         m_pcTagDetectorAlgorithm = m_pcCamera->GetAlgorithm<CCI_CamerasSensorTagDetectorAlgorithm>("duovero_camera", "tag_detector");
      }
      /* window configuration */
      m_pcScene = new QGraphicsScene(0.0f,
                                     0.0f,
                                     m_pcCamera->GetDescriptors()[0].HorizontalResolution,
                                     m_pcCamera->GetDescriptors()[0].VerticalResolution);
   

      /* camera sensor algorithms */   
      m_pcViewport = new QGraphicsView(m_pcScene);
      m_pcViewport->scale(0.5,0.5);
      m_pcLayout = new QVBoxLayout;
      m_pcLogTargetsBuffer = new QTextEdit;
      m_pcLogStatesBuffer = new QTextEdit;
      {
         m_pcLogTargetsBuffer->setReadOnly(true);
         QString strContents;
         QTextStream(&strContents) << "<b>" << "" << "</b>";
         m_pcLogTargetsBuffer->append(strContents); /* Write something in the buffer */
      }
      {
         m_pcLogStatesBuffer->setReadOnly(true);
         QString strContents;
         QTextStream(&strContents) << "<b>" << "" << "</b>";
         m_pcLogStatesBuffer->append(strContents); /* Write something in the buffer */
      }

      m_pcLayout->addWidget(m_pcViewport);
      m_pcLayout->addWidget(m_pcLogStatesBuffer);
      m_pcLayout->addWidget(m_pcLogTargetsBuffer);

      m_pcLogStatesStream = new CQTOpenGLLogStream(m_pcController->m_mapLogs["states"], m_pcLogStatesBuffer);
      m_pcLogTargetsStream = new CQTOpenGLLogStream(m_pcController->m_mapLogs["targets"], m_pcLogTargetsBuffer);

      setLayout(m_pcLayout);
      /* Draw sensor readings */
      Update();
   }


   /********************************************************************************/
   /********************************************************************************/

   CQtRobotUserInterfaceWidget::~CQtRobotUserInterfaceWidget() {
      delete m_pcLayout;
      delete m_pcViewport;
      delete m_pcScene;
      delete m_pcLogTargetsBuffer;
      delete m_pcLogStatesBuffer;
      delete m_pcLogTargetsStream;
      delete m_pcLogStatesStream;
   }

   /********************************************************************************/
   /********************************************************************************/

   void CQtRobotUserInterfaceWidget::Update() {
      m_pcScene->clear();
      m_pcViewport->setBackgroundBrush(QBrush(Qt::white, Qt::SolidPattern));

      const Real fEllipseSize = 7.5f;
      if(m_pcLEDDetectorAlgorithm != nullptr) {
         const CCI_CamerasSensorLEDDetectorAlgorithm::SReading::TList& sReadings =
            m_pcLEDDetectorAlgorithm->GetReadings();      
         for(UInt32 i = 0; i < sReadings.size(); ++i) {
            QPen cPen(Qt::black);
            m_pcScene->addEllipse(sReadings[i].Center.GetX() - (fEllipseSize * 0.5f),
                                  sReadings[i].Center.GetY() - (fEllipseSize * 0.5f),
                                  fEllipseSize,
                                  fEllipseSize,
                                  cPen);
         }
      }
      if(m_pcTagDetectorAlgorithm != nullptr) {
         const CCI_CamerasSensorTagDetectorAlgorithm::SReading::TList& sReadings =
            m_pcTagDetectorAlgorithm->GetReadings();      
         for(UInt32 i = 0; i < sReadings.size(); ++i) {
            QPen cPen(Qt::black);      
            m_pcScene->addEllipse(sReadings[i].Center.GetX() - (fEllipseSize * 0.5f),
                                  sReadings[i].Center.GetY() - (fEllipseSize * 0.5f),
                                  fEllipseSize,
                                  fEllipseSize,
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

}

