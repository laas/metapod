INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/benchmark)

ADD_LIBRARY(sample_3_dof SHARED models/sample_3_dof/sample_3_dof.cc)
ADD_LIBRARY(sample_7_dof SHARED models/sample_7_dof/sample_7_dof.cc)
ADD_LIBRARY(sample_15_dof SHARED models/sample_15_dof/sample_15_dof.cc)
ADD_LIBRARY(sample_31_dof SHARED models/sample_31_dof/sample_31_dof.cc)
ADD_LIBRARY(sample_63_dof SHARED models/sample_63_dof/sample_63_dof.cc)

ADD_EXECUTABLE(benchmark benchmark.cc)
TARGET_LINK_LIBRARIES(benchmark
  sample_3_dof
  sample_7_dof
  sample_15_dof
  sample_31_dof
  sample_63_dof
  simple_humanoid
)
