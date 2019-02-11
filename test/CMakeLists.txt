####################################################################################################
# utility {

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)

# }

####################################################################################################
# devices test {

include(test_project)
find_test_srcs()
build_exe(SRCS ${SOURCES} LIBS ${PROJECT_NAME} SUFFIX "-tests")

# } devices test
