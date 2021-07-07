find_program (SOCAT NAMES socat NO_CACHE)

if (NOT SOCAT)
  message(FATAL_ERROR "socat not found. Use: sudo apt install socat")
else ()
  message(STATUS "socat found: ${SOCAT}")
endif ()

setup_dep(utility $ENV{UTILITY_HOME} SUB_DIR "") 

include(test_project)
find_test_srcs()
build_tests(
  SRCS ${SOURCES}
  LIBS ${PROJECT_NAME} ${BTR_LIBS}
  SUFFIX "-tests"
  INC_DIRS ${utility_INC_DIR})

set(DOXYGEN_WARN NO)
set(DOXYGEN_PREPROCESSING YES)
build_doc(DEP ${PROJECT_NAME}-tests SUFFIX "-docs")
