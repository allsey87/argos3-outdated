/* -*- Mode: C++ -*-
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/**
 * @file <argos2/simulator/physics_engines/dynamics3d-bullet/dynamics3d_floor.h>
 *
 * @author Michael Allwright - <michael.allwright@upb.de>
 */

#ifndef DYNAMICS3D_FLOOR_H
#define DYNAMICS3D_FLOOR_H

#include <argos2/simulator/physics_engines/dynamics3d-bullet/dynamics3d_entity.h>
#include <argos2/simulator/physics_engines/dynamics3d-bullet/dynamics3d_engine.h>
#include <argos2/simulator/space/entities/floor_entity.h>


namespace argos {

   class CDynamics3DFloor : public CDynamics3DEntity {

   public:
      
      CDynamics3DFloor(CDynamics3DEngine& c_engine,
                     CFloorEntity& c_floor);
      virtual ~CDynamics3DFloor() {}
      
      virtual void UpdateEntityStatus();
      virtual void UpdateFromEntityStatus();

   private:

      CFloorEntity&                m_cFloorEntity;
   };

}

#endif
