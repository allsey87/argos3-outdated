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
 * @file <argos2/simulator/physics_engines/dynamics3d-bullet/dynamics3d_floor.cpp>
 *
 * @author Michael Allwright - <michael.allwright@upb.de>
 */

#include "dynamics3d_floor.h"
#include <argos2/simulator/space/entities/embodied_entity.h>
#include <argos2/simulator/physics_engines/dynamics3d-bullet/dynamics3d_engine.h>
#include <btBulletDynamicsCommon.h>

namespace argos {
   
   /****************************************/
   /****************************************/

   CDynamics3DFloor::CDynamics3DFloor(CDynamics3DEngine& c_engine, CFloorEntity& c_floor) :
      CDynamics3DEntity(c_engine, c_floor.GetEmbodiedEntity()),
      m_cFloorEntity(c_floor) {
      
      m_pcPrincipleCollisionShape = 
         new btStaticPlaneShape(btVector3(0.0f, 1.0f, 0.0f), 0.0f);
      m_pcPrincipleMotionState =
         new btDefaultMotionState(btTransform::getIdentity());
      m_pcPrincipleRigidBody = 
         new btRigidBody(TRigidBodyData(0.0f, m_pcPrincipleMotionState, m_pcPrincipleCollisionShape, btVector3(0.0f, 0.0f, 0.0f)));
   }

   /****************************************/
   /****************************************/

   /* override UpdateEntityStatus, updating floor status not implemented */
   void CDynamics3DFloor::UpdateEntityStatus() {}

   /****************************************/
   /****************************************/
   
   /* override UpdateFromEntityStatus, reading floor status not implemented */
   void CDynamics3DFloor::UpdateFromEntityStatus() {}

}
