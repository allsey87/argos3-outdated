/**
 * @file <argos3/plugins/simulator/entities/terrain_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef TERRAIN_ENTITY_H
#define TERRAIN_ENTITY_H

namespace argos {
   class CTerrainEntity;
}

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   class CTerrainEntity : public CComposableEntity {

   public:
   
      struct SHeightMap {
         UInt32 GridSize;
         UInt8* Data;
         Real GridScaleHeight;
         Real MinimumHeight;
         Real MaximumHeight;
      };
   
   public:

      ENABLE_VTABLE();

      CTerrainEntity();

      CTerrainEntity(const std::string& str_id,
                    const CVector3& c_position,
                    const CQuaternion& c_orientation);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline const CEmbodiedEntity& GetEmbodiedEntity() const {
         return *m_pcEmbodiedEntity;
      }
      
      inline SHeightMap& GetHeightMap() {
         return *m_psHeightMap;
      }

      inline const SHeightMap& GetHeightMap() const {
         return *m_psHeightMap;
      }

      virtual std::string GetTypeDescription() const {
         return "terrain";
      }

   private:

      CEmbodiedEntity*      m_pcEmbodiedEntity;
      SHeightMap*           m_psHeightMap;

   };

}

#endif
