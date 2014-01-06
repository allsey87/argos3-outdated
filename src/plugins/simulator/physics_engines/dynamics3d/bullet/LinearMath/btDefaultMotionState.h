#ifndef BT_DEFAULT_MOTION_STATE_H
#define BT_DEFAULT_MOTION_STATE_H

#include "btMotionState.h"
#include <cstdio>

///The btDefaultMotionState provides a common implementation to synchronize world transforms with offsets.
ATTRIBUTE_ALIGNED16(struct)	btDefaultMotionState : public btMotionState
{
	btTransform m_graphicsWorldTrans;
	btTransform	m_centerOfMassOffset;
	btTransform m_startWorldTrans;
	void*		m_userPointer;

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btDefaultMotionState(const btTransform& startTrans = btTransform::getIdentity(),const btTransform& centerOfMassOffset = btTransform::getIdentity())
		: m_graphicsWorldTrans(startTrans),
		m_centerOfMassOffset(centerOfMassOffset),
		m_startWorldTrans(startTrans),
		m_userPointer(0)

	{
	}

	///synchronizes world transform from user to physics
	virtual void	getWorldTransform(btTransform& centerOfMassWorldTrans ) const 
	{
			centerOfMassWorldTrans = m_graphicsWorldTrans * m_centerOfMassOffset.inverse();
			fprintf(stderr, "Reading motionstate via getWorldTransform centerOfMassWorldTrans = [%f %f %f]\n", centerOfMassWorldTrans.getOrigin().getX(), centerOfMassWorldTrans.getOrigin().getY(), centerOfMassWorldTrans.getOrigin().getZ());
			
	}

	///synchronizes world transform from physics to user
	///Bullet only calls the update of worldtransform for active objects
	virtual void	setWorldTransform(const btTransform& centerOfMassWorldTrans)
	{
			m_graphicsWorldTrans = centerOfMassWorldTrans * m_centerOfMassOffset ;
						fprintf(stderr, "Writing motionstate via setWorldTransform m_graphicsWorldTrans = [%f %f %f]\n", m_graphicsWorldTrans.getOrigin().getX(), m_graphicsWorldTrans.getOrigin().getY(), m_graphicsWorldTrans.getOrigin().getZ());
			
	}
	
	virtual void reset()
	{
	     fprintf(stderr, "reseting motion state\n");
	     m_graphicsWorldTrans = m_startWorldTrans;
	}

	

};

#endif //BT_DEFAULT_MOTION_STATE_H
