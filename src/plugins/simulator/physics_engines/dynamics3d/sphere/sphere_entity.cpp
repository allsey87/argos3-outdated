/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/sphere/sphere_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "sphere_entity.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>

namespace argos {

   /****************************************/
   /****************************************/

   CSphereEntity::CSphereEntity() :
      CComposableEntity(NULL),
      m_pcEmbodiedEntity(NULL),
      m_pcLEDEquippedEntity(NULL),
      m_fMass(1.0f) {
   }

   /****************************************/
   /****************************************/

   CSphereEntity::CSphereEntity(const std::string& str_id,
                                const CVector3& c_position,
                                const CQuaternion& c_orientation,
                                bool b_movable,
                                Real f_radius,
                                Real f_mass) :
      CComposableEntity(NULL, str_id),
      m_pcEmbodiedEntity(
         new CEmbodiedEntity(this,
                             str_id,
                             c_position,
                             c_orientation,
                             b_movable)),
      m_pcLEDEquippedEntity(
         new CLEDEquippedEntity(this,
                                str_id,
                                m_pcEmbodiedEntity)),
      m_fMass(f_mass) {
      AddComponent(*m_pcEmbodiedEntity);
      AddComponent(*m_pcLEDEquippedEntity);
   }

   /****************************************/
   /****************************************/

   void CSphereEntity::Init(TConfigurationNode& t_tree) {
      try {
         /* Init parent */
         CComposableEntity::Init(t_tree);
         /* Parse XML to get the radius */
         GetNodeAttribute(t_tree, "radius", m_fRadius);
         /* Parse XML to get the movable attribute */
         bool bMovable;
         GetNodeAttribute(t_tree, "movable", bMovable);
         if(bMovable) {
            /* Parse XML to get the mass */
            GetNodeAttribute(t_tree, "mass", m_fMass);
         }
         else {
            m_fMass = 0.0f;
         }
         /* Create embodied entity using parsed data */
         m_pcEmbodiedEntity = new CEmbodiedEntity(this);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(GetNode(t_tree, "body"));
         m_pcEmbodiedEntity->SetMovable(bMovable);
         /* Init LED equipped entity component */
         m_pcLEDEquippedEntity = new CLEDEquippedEntity(this,
                                                        GetId() + ".leds",
                                                        m_pcEmbodiedEntity);
         AddComponent(*m_pcLEDEquippedEntity);
         if(NodeExists(t_tree, "leds")) {
            m_pcLEDEquippedEntity->Init(GetNode(t_tree, "leds"));
         }
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize the sphere entity.", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CSphereEntity::Reset() {
      /* Reset all components */
      m_pcEmbodiedEntity->Reset();
      m_pcLEDEquippedEntity->Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CSphereEntity,
                   "sphere",
                   "1.0",
                   "Michael Allwright [allsey87@gmail.com]",
                   "A sphere.",
                   "The sphere entity can be used to model obstacles or sphere-shaped\n"
                   "grippable objects. The sphere has a red LED on the center of one\n"
                   "of the circular surfaces, that allows perception using the cameras.\n"
                   "The sphere can be movable or not. A movable object can be pushed\n"
                   "and gripped. An unmovable object is pretty much like a column.\n\n"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "To declare an unmovable object (i.e., a column) you need the following:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <sphere id=\"cyl1\"\n"
                   "              position=\"0.4,2.3,0\"\n"
                   "              orientation=\"45,90,0\"\n"
                   "              radius=\"0.8\"\n"
                   "              movable=\"false\" />\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "To declare a movable object you need the following:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <sphere id=\"cyl1\"\n"
                   "              position=\"0.4,2.3,0\"\n"
                   "              orientation=\"45,90,0\"\n"
                   "              radius=\"0.8\"\n"
                   "              movable=\"true\"\n"
                   "              mass=\"2.5\" />\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'position' attribute specifies the position of the base of the sphere\n"
                   "in the arena. The three values are in the X,Y,Z order.\n"
                   "The 'orientation' attribute specifies the orientation of the sphere. All\n"
                   "rotations are performed with respect to the center of mass. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees.\n"
                   "The 'radius' attribute specifies the size of the sphere. When\n"
                   "you add a sphere, imagine it initially unrotated and positioned on the origin.\n"
                   "The 'movable' attribute specifies whether or not the object is movable. When\n"
                   "set to 'false', the object is unmovable: if another object pushes against it,\n"
                   "the sphere won't move. When the attribute is set to 'true', the sphere is\n"
                   "movable upon pushing or gripping. When an object is movable, the 'mass'\n"
                   "attribute is required.\n"
                   "The 'mass' attribute quantifies the mass of the sphere in kg.\n\n"
                   "OPTIONAL XML CONFIGURATION\n\n"
                   "It is possible to add any number of colored LEDs to the sphere. In this way,\n"
                   "the sphere is visible with a robot camera. The position and color of the\n"
                   "LEDs is specified with the following syntax:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <sphere id=\"box1\"\n"
                   "              position=\"0.4,2.3,0\"\n"
                   "              orientation=\"45,90,0\"\n"
                   "              radius=\"0.8\"\n"
                   "              movable=\"true\"\n"
                   "              mass=\"2.5\">\n"
                   "      <leds>\n"
                   "        <led position=\"0.81,0,0\" color=\"white\" />\n"
                   "        <led position=\"-0.81,0,0\" color=\"red\" />\n"
                   "        <led position=\"0,0.81,0\" color=\"blue\" />\n"
                   "        <led position=\"0,-0.81,0\" color=\"green\" />\n"
                   "      </leds>\n"
                   "    </sphere>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "In the example, four LEDs are added around the sphere. The LEDs have\n"
                   "different colors and are located around the sphere.\n",
                   "Usable"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CSphereEntity);

   /****************************************/
   /****************************************/

}
