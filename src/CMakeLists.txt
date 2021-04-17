add_library(myslam SHARED
        loopclosing.cpp
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        OrbExtractor.cpp
        OrbMatcher.cpp)

target_link_libraries(myslam
        ${THIRD_PARTY_LIBS})
