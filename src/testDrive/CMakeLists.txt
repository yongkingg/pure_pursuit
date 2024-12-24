 cmake_minimum_required(VERSION 3.0.2)
project(testDrive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  morai_msgs
  message_generation # 커스텀 메세지 활용을 위해 필수적으로 추가.
)

#==================커스텀 메세지 사용 시==================#
add_message_files(
  FILES
  carInfo.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs  # 사용할 다른 패키지의 메시지 타입을 나열
)
#=====================================================#

#===================커스텀 서비스 사용 시===================#
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
#=======================================================#

#====================커스텀 액션 사용 시====================#
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
#=======================================================#

# ========현재 패키지가 의존하는 다른 ROS 패키지 명시========
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime morai_msgs
)
# ===================================================

# include 디렉토리에서 헤더파일 가져올때 include 주석 제거 
include_directories(
include
${catkin_INCLUDE_DIRS}
)

#==================Cpp 파일 제작 시==================#
## 1. 실행 파일 선언 - main.cpp 파일을 컴파일할 실행 파일을 정의함
add_executable(${PROJECT_NAME}_makeCsv src/makeCsv.cpp)
add_executable(${PROJECT_NAME}_control src/control.cpp)
add_executable(${PROJECT_NAME}_gps src/gps.cpp)


## 2. 종속성 설정 - 이 실행 파일이 필요한 라이브러리나 패키지에 링크되도록 설정함

target_link_libraries(${PROJECT_NAME}_makeCsv
    ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_control
    ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_gps
    ${catkin_LIBRARIES}
    Geographic
)
## 3. 의존성 설정 - 필요한 타겟 간의 의존성을 설정함.
add_dependencies(${PROJECT_NAME}_makeCsv ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_gps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#===================================================#

#====================라이브러리 제작====================#
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/package.cpp
# )
#====================================================#

