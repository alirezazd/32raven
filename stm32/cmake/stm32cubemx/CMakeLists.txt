cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms
	STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core
)

# Vendored CMSIS — included SYSTEM so the compiler and clangd treat it as an
# external library: no warnings, no clang-tidy, no naming checks on its headers.
set(MX_System_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/CMSIS_F4xx
)

# STM32CubeMX generated application sources
set(MX_Application_Src

    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f407xx.s
)

# Drivers Midllewares



# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    ${TOOLCHAIN_LINK_LIBRARIES}
    
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_include_directories(stm32cubemx SYSTEM INTERFACE ${MX_System_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
