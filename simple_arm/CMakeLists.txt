INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set header files.
SET(METAPOD_SIMPLE_ARM_MODEL_HEADERS
  include/metapod/models/simple_arm/simple_arm.hh
  include/metapod/models/simple_arm/config.hh
  )

# Set header files installation prefix.
INSTALL(FILES ${METAPOD_SIMPLE_ARM_MODEL_HEADERS}
  DESTINATION include/metapod/models/simple_arm
  )

ADD_LIBRARY(metapod_simple_arm SHARED
  include/metapod/models/simple_arm/simple_arm.cc
  ${METAPOD_SIMPLE_ARM_MODEL_HEADERS})
