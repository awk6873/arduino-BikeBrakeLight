cmake_minimum_required(VERSION 3.13)

project( Neuton_KWS_Frontend
    VERSION 1.0
    DESCRIPTION "This project is examples of Keyword spotting audio frontend implementation"
    LANGUAGES
        C
)

add_library(${PROJECT_NAME}
    neuton_kws_frontend.c
)

add_library(Neuton::KWSFrontend ALIAS ${PROJECT_NAME})

target_include_directories (${PROJECT_NAME}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Neuton::DSP::Support::Windowing
        Neuton::DSP::Support::Scaling::MinMax
        Neuton::DSP::Statistic::MinMax
        Neuton::DSP::Transform
        Neuton::Utils::SlidingWindow
        m
)