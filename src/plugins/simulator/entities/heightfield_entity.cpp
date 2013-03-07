/**
 * @file <argos3/plugins/simulator/entities/heightfield_entity.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "heightfield_entity.h"
#include <argos3/core/utility/math/matrix/rotationmatrix3.h>
#include <argos3/core/simulator/space/space.h>

namespace argos {

   class CHeightFieldEmbodiedEntity : public CEmbodiedEntity {

   public:

    	CHeightFieldEmbodiedEntity(CHeightFieldEntity* pc_parent) :
         CEmbodiedEntity(pc_parent),
         m_cHalfSize(pc_parent->GetHeightMap().GridSizeX * pc_parent->GetHeightMap().GridScaling.GetX(),
                     pc_parent->GetHeightMap().GridSizeY * pc_parent->GetHeightMap().GridScaling.GetY(),
                     pc_parent->GetHeightMap().GridHeightScale * (pc_parent->GetHeightMap().MaximumHeight - pc_parent->GetHeightMap().MinimumHeight)) {}

    	CHeightFieldEmbodiedEntity(CHeightFieldEntity* pc_parent,
                         const std::string& str_id,
                         const CVector3& c_position,
                         const CQuaternion& c_orientation):
         CEmbodiedEntity(pc_parent,
                         str_id,
                         c_position,
                         c_orientation,
                         false),
         m_cHalfSize(pc_parent->GetHeightMap().GridSizeX * pc_parent->GetHeightMap().GridScaling.GetX(),
                     pc_parent->GetHeightMap().GridSizeY * pc_parent->GetHeightMap().GridScaling.GetY(),
                     pc_parent->GetHeightMap().GridHeightScale * (pc_parent->GetHeightMap().MaximumHeight - pc_parent->GetHeightMap().MinimumHeight)) {}

   protected:

      virtual void CalculateBoundingBox() {
         m_cOrientationMatrix.SetFromQuaternion(GetOrientation());
         CalculateBoundingBoxFromHalfSize(GetBoundingBox(),
                                          m_cHalfSize,
                                          GetPosition(),
                                          m_cOrientationMatrix);
      }

   private:

      CVector3 m_cHalfSize;
      CRotationMatrix3 m_cOrientationMatrix;

   };

   /****************************************/
   /****************************************/

   CHeightFieldEntity::CHeightFieldEntity():
      CComposableEntity(NULL),
      m_pcEmbodiedEntity(NULL) {}

   /****************************************/
   /****************************************/

   CHeightFieldEntity::CHeightFieldEntity(const std::string& str_id,
                                          const CVector3& c_position,
                                          const CQuaternion& c_orientation,
                                          UInt32 un_grid_size_x,
                                          UInt32 un_grid_size_y,
                                          Real f_granularity_x,
                                          Real f_granularity_y,
                                          Real f_grid_height_scale,
                                          const CRange<Real>& c_height_range) :
      SHeightMap(un_grid_size_x,
                 un_grid_size_y,
                 f_granularity_x,
                 f_granularity_y,
                 f_grid_height_scale,
                 c_height_range),
      CComposableEntity(NULL, str_id),
      m_pcEmbodiedEntity(new CBoxEmbodiedEntity(this,
                                                str_id,
                                                c_position,
                                                c_orientation)) {
      AddComponent(*m_pcEmbodiedEntity);
   }

   /****************************************/
   /****************************************/

   void CHeightFieldEntity::Init(TConfigurationNode& t_tree) {
      try {
         
         /* Init parent */
         CComposableEntity::Init(t_tree);
         
         /* Parse XML to get the grid size */
         std::string strGridSize;
         GetNodeAttribute(t_tree, "size", strGridSize);         
         UInt32 punGridSize[2];
         ParseValues(strGridSize, 2, punGridSize, ',');
         SHeightMap.GridSizeX = punGridSize[0];
         SHeightMap.GridSizeY = punGridSize[1];
         
         /* Parse XML to get the grid granularity */
         std::string strGridGranularity;
         GetNodeAttribute(t_tree, "granularity", strGridGranularity);         
         UInt32 punGridGranularity[2];
         ParseValues(strGridGranularity, 2, punGridGranularity, ',');
         SHeightMap.GranularityX = punGridGranularity[0];
         SHeightMap.GranularityY = punGridGranularity[1];
         
         /* Parse XML to get the height scale */
         GetNodeAttribute(t_tree, "height_scale", SHeightMap.HeightScale);
           
         /* Parse XML to get the height range */
         GetNodeAttribute(t_tree, "height_range", SHeightMap.HeightRange);
         
         /* Parse XML to get the source */
         std::string strSource, strData;
         GetNodeAttribute(t_tree, "source", strSource);
         
         if(strSource == "inline") {
            GetNodeText(GetNode(t_tree, "data"), strData);
         }
         else if(strSource == "file") {
            std::ifstream()
         }
         else THROW_ARGOSEXCEPTION("Unrecognized height field source \"" + strSource + "\"");
         
         
         
         /* Create embodied entity using parsed data */
         m_pcEmbodiedEntity = new CBoxEmbodiedEntity(this, m_cSize);
         AddComponent(*m_pcEmbodiedEntity);
         m_pcEmbodiedEntity->Init(t_tree);
         m_pcEmbodiedEntity->SetMovable(bMovable);
         
         
         UpdateComponents();
      }
      catch(CARGoSException& ex) {
         THROW_ARGOSEXCEPTION_NESTED("Failed to initialize height field entity \"" << GetId() << "\".", ex);
      }
   }

   /****************************************/
   /****************************************/

   void CHeightFieldEntity::Reset() {
      /* Reset all components */
      m_pcEmbodiedEntity->Reset();
      /* Update components */
      UpdateComponents();
   }

   /****************************************/
   /****************************************/

   REGISTER_ENTITY(CHeightFieldEntity,
                   "heightfield",
                   "1.0",
                   "Michael Allwright [allsey87@gmail.com]",
                   "A region of area whose terrain is controlled by a height map",
                   "A Height Field is an positionable region in the arena that represents an uneven surface"
                   "REQUIRED XML CONFIGURATION\n\n"
                   "To declare an unmovable object (i.e., a wall) you need the following:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <box id=\"TODO\"\n" 
                   "    ...\n"
                   "  </arena>\n\n"
                   "To declare a movable object you need the following:\n\n"
                   "  <arena ...>\n"
                   "    ...\n"
                   "    <box id=\"TODO\"/>\n"
                   "    ...\n"
                   "  </arena>\n\n"
                   "The 'id' attribute is necessary and must be unique among the entities. If two\n"
                   "entities share the same id, initialization aborts.\n"
                   "The 'position' attribute specifies the position of the center of mass of the\n"
                   "height field in the arena. The three values are in the X,Y,Z order.\n"
                   "The 'orientation' attribute specifies the orientation of the height field. All\n"
                   "rotations are performed with respect to the center of mass. The order of the\n"
                   "angles is Z,Y,X, which means that the first number corresponds to the rotation\n"
                   "around the Z axis, the second around Y and the last around X. This reflects\n"
                   "the internal convention used in ARGoS, in which rotations are performed in\n"
                   "that order. Angles are expressed in degrees.\n",
                   "Usable"
      );

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_SPACE_OPERATIONS_ON_COMPOSABLE(CHeightFieldEntity);

   /****************************************/
   /****************************************/

}
