add_compile_options(-std=c++11)
project(test)
aux_source_directory(. DIR_SRCS)
add_executable(test ${DIR_SRCS})

