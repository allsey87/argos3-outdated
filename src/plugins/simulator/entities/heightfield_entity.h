/**
 * @file <argos3/plugins/simulator/entities/terrain_entity.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef TERRAIN_ENTITY_H
#define TERRAIN_ENTITY_H

namespace argos {
   class CHeightFieldEntity;
}

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/core/config.h>

namespace argos {

   class CHeightFieldEntity : public CComposableEntity {

   public:

      ENABLE_VTABLE();

   public:
   
      struct SHeightMap {
         UInt32 GridSizeX;
         UInt32 GridSizeY;
         Real GranularityX;
         Real GranularityY;
         Real HeightScale;
         CRange<Real> HeightRange;
         CByteArray Data;
         
         SHeightMap(UInt32 un_grid_size_x,
                    UInt32 un_grid_size_y,
                    Real f_granularity_x,
                    Real f_granularity_y,
                    Real f_height_scale,
                    const CRange<Real>& c_height_range) :
            GridSizeX(un_grid_size_x),
            GridSizeY(un_grid_size_y),
            GranularityX(f_granularity_x),
            GranularityY(f_granularity_y),
            HeightScale(f_height_scale),
            HeightRange(c_height_range),
#ifdef ARGOS_USE_DOUBLE
            Data(8u * un_grid_size_x * un_grid_size_y, 0u) {}
#else
            Data(4u * un_grid_size_x * un_grid_size_y, 0u) {}
#endif            
      };
   
      CHeightFieldEntity();

      CHeightFieldEntity(const std::string& str_id,
                         const CVector3& c_position,
                         const CQuaternion& c_orientation
                         const SHeightMap& s_height_map);

      virtual void Init(TConfigurationNode& t_tree);
      virtual void Reset();

      inline CEmbodiedEntity& GetEmbodiedEntity() {
         return *m_pcEmbodiedEntity;
      }

      inline const CEmbodiedEntity& GetEmbodiedEntity() const {
         return *m_pcEmbodiedEntity;
      }
      
      inline SHeightMap& GetHeightMap() {
         return m_pHeightMap;
      }

      inline const SHeightMap& GetHeightMap() const {
         return m_pHeightMap;
      }

      virtual std::string GetTypeDescription() const {
         return "heightfield";
      }

   private:
      SHeightMap            m_sHeightMap;
      
      CEmbodiedEntity*      m_pcEmbodiedEntity;
   };

}

#endif
