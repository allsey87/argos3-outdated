/**
 * @file <argos3/plugins/robot/prototype/simulator/forwards_camera_equipped_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */


#ifndef FORWARDS_CAMERA_EQUIPPED_ENTITY_H
#define FORWARDS_CAMERA_EQUIPPED_ENTITY_H

namespace argos {
   class CForwardsCameraEquippedEntity;
   class CEmbodiedEntity;
}

#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/utility/math/quaternion.h>

#include <argos3/plugins/robots/prototype/simulator/forwards_camera_entity.h>

namespace argos {

   class CForwardsCameraEquippedEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:

      /**
       * Class constructor.
       * This constructor is meant to be used with the Init() method.
       * @param pc_parent The parent of this entity.
       */
      CForwardsCameraEquippedEntity(CComposableEntity* pc_parent);

      /**
       * Class constructor.
       * This constructor is meant to be standalone.
       * You should not call Init() after using this constructor, or
       * memory leaks are likely to happen.
       * @param pc_parent The parent of this entity.
       * @param str_id The id of this entity.
       */
       CForwardsCameraEquippedEntity(CComposableEntity* pc_parent,
                                     const std::string& str_id);
         

      /**
       * Initializes the state of the entity from the XML configuration tree.
       * @throws CARGoSException if a parse error occurs
       */
      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "forwards_camera_container";
      }

      const CForwardsCameraEntity::TList& GetAllForwardsCameras() const {
         return m_tForwardsCameras;
      }

      const CQuaternion& GetOffsetOrientation(UInt32 un_index) const;

      const CVector3& GetOffsetPosition(UInt32 un_index) const;

      const CPositionalEntity& GetPositionalEntity(UInt32 un_index) const;
      
      CForwardsCameraEntity& GetForwardsCamera(UInt32 un_index);

   private:

      CForwardsCameraEntity::TList m_tForwardsCameras;

      /* Links to the positional entity that the forwards cameras are attached to */
      std::vector<CPositionalEntity*> m_vecPositionalEntities;
      std::vector<CVector3> m_vecPositionOffsets;
      std::vector<CQuaternion> m_vecOrientationOffsets;

   };
}

#endif
