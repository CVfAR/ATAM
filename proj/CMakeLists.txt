##############################
# Project
##############################

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
file(GLOB subdirs RELATIVE ${dir} ${dir}/*)

foreach(subdir ${subdirs})
    if(IS_DIRECTORY ${dir}/${subdir})
        set(buildFlag build${subdir})
        option(${buildFlag} "Build ${subdir}" ON)
        if(${buildFlag})
            message("subdir: ${subdir}")
            add_subdirectory(${subdir})
        endif()
    endif()
endforeach()
