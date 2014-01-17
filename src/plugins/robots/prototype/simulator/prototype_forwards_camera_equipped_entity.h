/**
 * @file <argos3/plugins/robot/prototype/simulator/prototype_forwards_camera_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


#ifndef PROTOTYPE_FORWARDS_CAMERA_EQUIPPED_ENTITY_H
#define PROTOTYPE_FORWARDS_CAMERA_EQUIPPED_ENTITY_H

namespace argos {
   class CPrototypeForwardsCameraEquippedEntity;
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

namespace argos {

   class CPrototypeForwardsCameraEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();

   public:

      /**
       * Class constructor.
       * This constructor is meant to be used with the Init() method.
       * @param pc_parent The parent of this entity.
       */
      CPrototypeForwardsCameraEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * This constructor is meant to be standalone.
       * You should not call Init() after using this constructor, or
       * memory leaks are likely to happen.
       * @param pc_parent The parent of this entity.
       * @param str_id The id of this entity.
       * @param c_aperture The aperture of the visibility cone
       * @param c_offset The positional offset of this omnidirectionalcamera with respect to the robot reference point.
       */
      /*CPrototypeForwardsCameraEquippedEntity(CComposableEntity* pc_parent,
                                             CPositionalEntity* pc_positional,
                                             const std::string& str_id,
                                             const CQuaternion& c_offset_orientation,
                                             const CVector3&    c_offset_position);
      */

      /**
       * Initializes the state of the entity from the XML configuration tree.
       * @throws CARGoSException if a parse error occurs
       */
      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "prototype_forwards_camera";
      }

      const CQuaternion& GetOffsetOrientation() const {
         return m_cOffsetOrientation;
      }

      const CVector3& GetOffsetPosition() const {
         return m_cOffsetPosition;
      }

      const CRadians& GetFieldOfView() const {
         return m_cFieldOfView;
      }

      Real GetRange() const {
         return m_fRange;
      }

      const CVector2& GetSensorResolution() const {
         return m_cSensorResolution;
      }

      //make const
      CPositionalEntity* GetPositionalEntity() {
         return m_pcPositionalEntity;
      }

      /* TEMP / TESTING */
      CVector3 SphereCenter;
      Real SphereRadius;


   private:

      /** The positional offset of this omnidirectionalcamera with respect to the robot reference point */
      CVector3 m_cOffsetPosition;
      CQuaternion m_cOffsetOrientation; /* to vectors of struct <quat, vec, positional?> */
      CRadians m_cFieldOfView;
      Real m_fRange;
      CVector2 m_cSensorResolution;
      
      /* we should ever read from this pointer, make it const */
      CPositionalEntity* m_pcPositionalEntity;



   };
}

#endif
