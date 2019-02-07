####################################################################################################
# utility {

set(UTILITY_HOME $ENV{UTILITY_HOME})
include_directories(${UTILITY_HOME}/include)

# }

####################################################################################################
# devices test {

set(EXE_SUFFIX "-tests")

include(test_project)

# } devices test
