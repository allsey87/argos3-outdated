# Find the Bullet Physics Engine Library
#
# The module defines these variables:
#
#  BULLET_INCLUDE_DIRS - Include directory of the Bullet Library
#  BULLET_LIBRARIES    - The location of the Bullet Library
#

# Initially assume bullet cannot be found
SET(BULLET_FOUND FALSE)

# Look for the header files
FIND_PATH(BULLET_INCLUDE_DIRS btBulletDynamicsCommon.h
  /usr/include/bullet
  /usr/local/include/bullet
)
IF(BULLET_INCLUDE_DIRS)
  MESSAGE(STATUS "Found Bullet include dir: ${BULLET_INCLUDE_DIRS}")
ELSE(BULLET_INCLUDE_DIRS)
  MESSAGE(STATUS "Could NOT find Bullet headers.")
ENDIF(BULLET_INCLUDE_DIRS)

# Look for the bullet libraries file
FIND_LIBRARY(BULLET_DYNAMICS_LIBRARY
  NAMES BulletDynamics
  PATHS 
  /usr/lib
  /usr/local/lib
)

FIND_LIBRARY(BULLET_COLLISION_LIBRARY
  NAMES BulletCollision
  PATHS
  /usr/lib
  /usr/local/lib
)

FIND_LIBRARY(BULLET_LINEARMATH_LIBRARY
  NAMES LinearMath 
  PATHS
  /usr/lib
  /usr/local/lib
)

IF(BULLET_DYNAMICS_LIBRARY AND BULLET_COLLISION_LIBRARY AND BULLET_LINEARMATH_LIBRARY)
   SET(BULLET_LIBRARIES ${BULLET_DYNAMICS_LIBRARY} ${BULLET_COLLISION_LIBRARY} ${BULLET_LINEARMATH_LIBRARY})
   MESSAGE(STATUS "Found Bullet libraries: ${BULLET_LIBRARIES}")
ELSE(BULLET_DYNAMICS_LIBRARY AND BULLET_COLLISION_LIBRARY AND BULLET_LINEARMATH_LIBRARY)
   MESSAGE(STATUS "Could NOT find Bullet libraries.")
ENDIF(BULLET_DYNAMICS_LIBRARY AND BULLET_COLLISION_LIBRARY AND BULLET_LINEARMATH_LIBRARY)

# If both were found, set the BULLET_FOUND to true
IF(BULLET_INCLUDE_DIRS AND BULLET_LIBRARIES)
   MESSAGE(STATUS "Bullet has been found")
   SET(BULLET_FOUND TRUE)
ENDIF(BULLET_INCLUDE_DIRS AND BULLET_LIBRARIES)

