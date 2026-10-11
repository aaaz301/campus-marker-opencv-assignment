cmake_minimum_required(VERSION 3.16)
project(mark_detector LANGUAGES CXX)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(mark_detector
    lib/core/timing.cpp
    lib/pipeline/diagnostics_context.cpp
    lib/pipeline/diagnostics_recorder.cpp
    lib/pipeline/run_report.cpp
    lib/core/marker_geometry.cpp
    lib/core/geometry_utils.cpp
    lib/config/config.cpp
    lib/preprocess/preprocess.cpp
    lib/geometry/geometry_matcher.cpp
    lib/geometry/geometry_observation.cpp
    lib/geometry/geometry_l_topology.cpp
    lib/geometry/geometry_anchor_evidence.cpp
    lib/geometry/geometry_validation.cpp
    lib/geometry/geometry_assignment_completion.cpp
    lib/corners/corner_resolver.cpp
    lib/corners/corner_observation.cpp
    lib/corners/corner_edge_fit.cpp
    lib/corners/corner_evidence_validation.cpp
    lib/corners/screen_order.cpp
    lib/corners/semantic_resolver.cpp
    lib/corners/detection_validator.cpp
    lib/corners/detection_publication.cpp
    lib/pipeline/decode_stage.cpp
    lib/pipeline/frame_sequence.cpp
    lib/pipeline/temporal_geometry.cpp
    lib/pipeline/temporal_stabilizer.cpp
    lib/pipeline/display_history.cpp
    lib/pipeline/stabilize_stage.cpp
    lib/pipeline/detector.cpp
)
target_include_directories(mark_detector
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)
target_link_libraries(mark_detector PUBLIC ${OpenCV_LIBS})

add_library(mark_offline
    app/offline_runner.cpp
    app/debug_renderer.cpp
    app/input_completion.cpp
    app/display_session.cpp
    app/video_exporter.cpp
    app/run_console.cpp
)
target_include_directories(mark_offline PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/app"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)
target_link_libraries(mark_offline PUBLIC mark_detector)
target_compile_definitions(mark_offline PRIVATE
    MARK_COMMIT="UNSPECIFIED"
    MARK_CODE_HASH="UNSPECIFIED"
    MARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)

add_executable(marker_app app/main.cpp)
target_link_libraries(marker_app PRIVATE mark_offline)

add_executable(marker_display_probe app/display_probe.cpp)
target_link_libraries(marker_display_probe PRIVATE ${OpenCV_LIBS})

add_executable(
    calibration_tool
    tools/calibration/calibration_main.cpp
    tools/calibration/calibration_config.cpp
    tools/calibration/calibration_video.cpp
    tools/calibration/calibration_detection.cpp
    tools/calibration/calibration_result.cpp
)
target_link_libraries(calibration_tool PRIVATE ${OpenCV_LIBS})