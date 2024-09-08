find(GLOB ex_headers "*.h")

install (FILES ${ex_headers} DESTINATION include/gtsamexamples)

gtsamAddExamplesGlob("examples/*.cpp" "" "${PROJECT_NAME}")

find(GLOB ex_srcs "*.cpp")

add_library(${PROJECT_NAME} SHARED ${ex_srcs} ${ex_headers})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} gtsam)