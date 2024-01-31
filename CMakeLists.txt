cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(LargePointCloudPreprocessing)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB_RECURSE LIBS include/preprocessing/*.h)

add_library(${PROJECT_NAME} src/LargePointCloudPreprocessing.cpp ${LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

### Tests
enable_testing()
include_directories(tests)

# Set the model to use in tests
set(TEST_CLOUD "${CMAKE_CURRENT_SOURCE_DIR}/assets/pca-modelo.pcd" CACHE STRING "Input point cloud for test")
if (NOT TEST_CLOUD)
    message(WARNING "No point cloud set for test, using default path: ../assets/pca-modelo.pcd")
endif()

# Crop Box
add_executable(TestApplyCropBox tests/ApplyCropBoxTest.cpp)
target_include_directories(TestApplyCropBox PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyCropBox ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyCropBoxTest COMMAND TestApplyCropBox)
target_compile_definitions(TestApplyCropBox PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Voxel Grid
add_executable(TestApplyVoxelGrid tests/ApplyVoxelGridTest.cpp)
target_include_directories(TestApplyVoxelGrid PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyVoxelGrid ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyVoxelGridTest COMMAND TestApplyVoxelGrid)
target_compile_definitions(TestApplyVoxelGrid PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Statistical Outlier Removal
add_executable(TestApplyStatisticalOutlierRemoval tests/ApplyStatisticalOutlierRemovalTest.cpp)
target_include_directories(TestApplyStatisticalOutlierRemoval PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyStatisticalOutlierRemoval ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyStatisticalOutlierRemovalTest COMMAND TestApplyStatisticalOutlierRemoval)
target_compile_definitions(TestApplyStatisticalOutlierRemoval PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Normal Estimation
add_executable(TestApplyNormalEstimation tests/ApplyNormalEstimationTest.cpp)
target_include_directories(TestApplyNormalEstimation PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyNormalEstimation ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyNormalEstimationTest COMMAND TestApplyNormalEstimation)
target_compile_definitions(TestApplyNormalEstimation PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Reescale
add_executable(TestApplyReescale tests/ApplyReescaleTest.cpp)
target_include_directories(TestApplyReescale PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyReescale ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyReescaleTest COMMAND TestApplyReescale)
target_compile_definitions(TestApplyReescale PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Centralization
add_executable(TestApplyCentralization tests/ApplyCentralizationTest.cpp)
target_include_directories(TestApplyCentralization PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyCentralization ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyCentralizationTest COMMAND TestApplyCentralization)
target_compile_definitions(TestApplyCentralization PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Alignment
add_executable(TestApplyAlignment tests/ApplyAlignmentTest.cpp)
target_include_directories(TestApplyAlignment PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyAlignment ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyAlignmentTest COMMAND TestApplyAlignment)
target_compile_definitions(TestApplyAlignment PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Noise Add
add_executable(TestApplyNoiseAdd tests/ApplyNoiseAddTest.cpp)
target_include_directories(TestApplyNoiseAdd PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyNoiseAdd ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyNoiseAddTest COMMAND TestApplyNoiseAdd)
target_compile_definitions(TestApplyNoiseAdd PRIVATE TEST_CLOUD="${TEST_CLOUD}")

# Cube Reescale
add_executable(TestApplyCubeReescale tests/ApplyCubeReescaleTest.cpp)
target_include_directories(TestApplyCubeReescale PUBLIC ${CMAKE_HOME_DIRECTORY}/include)
target_link_libraries(TestApplyCubeReescale ${PCL_LIBRARIES} ${libs} ${PROJECT_NAME})

add_test(NAME TestApplyCubeReescaleTest COMMAND TestApplyCubeReescale)
target_compile_definitions(TestApplyCubeReescale PRIVATE TEST_CLOUD="${TEST_CLOUD}")
