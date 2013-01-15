INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/include)

# Set header files.
SET(METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS
  include/metapod/models/simple_humanoid/body.hh
  include/metapod/models/simple_humanoid/joint.hh
  include/metapod/models/simple_humanoid/robot.hh
  include/metapod/models/simple_humanoid/simple_humanoid.hh
  include/metapod/models/simple_humanoid/config.hh
  )

# Set header files installation prefix.
INSTALL(FILES ${METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS}
  DESTINATION include/metapod/models/simple_humanoid
  )

ADD_LIBRARY(metapod_simple_humanoid SHARED
  include/metapod/models/simple_humanoid/simple_humanoid.cc
  ${METAPOD_SIMPLE_HUMANOID_MODEL_HEADERS})
