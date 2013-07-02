cmake_minimum_required(VERSION 2.6)
project(examples-hubo-motion-rt)

link_libraries(-lm -lach -lc -lhuboctrl -ldaemonizer -lrt -lhuboparams)

add_executable(ctrl-sample "src/ctrl-sample.cpp")
add_executable(ctrl-arm-sample "src/ctrl-arm-sample.cpp")
add_executable(ctrl-trajectory-sample "src/ctrl-trajectory-sample.cpp")
