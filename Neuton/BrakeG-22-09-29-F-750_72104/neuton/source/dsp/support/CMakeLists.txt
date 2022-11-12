cmake_minimum_required(VERSION 3.13)

project( Neuton_DSP_Support
    VERSION 1.0
    DESCRIPTION "Neuton DSP Support library."
    LANGUAGES
        C
)

add_subdirectory(clipping)
add_subdirectory(scaling)
add_subdirectory(quantization)
add_subdirectory(softfloat)
add_subdirectory(windowing)

add_library(${PROJECT_NAME} INTERFACE)
add_library(Neuton::DSP::Support ALIAS ${PROJECT_NAME})

target_include_directories (${PROJECT_NAME}
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        Neuton::DSP::Support::Quantization
        Neuton::DSP::Support::Clipping
        Neuton::DSP::Support::Scaling
        Neuton::DSP::Support::Float24
        Neuton::DSP::Support::Windowing
)