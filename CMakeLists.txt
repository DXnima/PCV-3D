# 最低cmake版本3.10
cmake_minimum_required( VERSION 3.10 )
# 设置项目名
project(PCLTest)
# 设置UTF-8
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
# 头文件和源文件
file(GLOB INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)
file(GLOB SOURCE  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# 设置VS显示头文件
source_group("Header Files" FILES ${INCLUDE})
# 设置头文件 include目录下
include_directories( include )
# 设置源文件 src目录下
aux_source_directory( src DIRS)
# 寻找PCL库
find_package( PCL REQUIRED )
if ( NOT PCL_FOUND )
  message(STATUS "This project requires the PCL library, and will not be compiled.")
  return()  
endif()
# 添加PCL相关库
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# 寻找Boost库
find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()  
endif()
include_directories(${Boost_INCLUDE_DIRS})
# 寻找CGAL库
find_package( CGAL QUIET COMPONENTS  )
if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()  
endif()
# 添加CGAL库
include(${CGAL_USE_FILE})
include_directories(${CGAL_INCLUDE_DIRS} )
# 设置编译类型
IF (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, options are: Release Release RelWithDebInfo MinSizeRel." FORCE)
ENDIF()
# 可执行文件
add_executable( 
	${PROJECT_NAME} 
	${INCLUDE} 
	${SOURCE}
)
# 链接PCL库
target_link_libraries( 
	${PROJECT_NAME} 
	${PCL_LIBRARIES} 
	${CGAL_LIBRARIES} 
	${Boost_LIBRARIES}	
	${CMAKE_THREAD_LIBS_INIT}
)