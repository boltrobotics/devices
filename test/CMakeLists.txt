find_program (SOCAT_EXE NAMES socat)

if (SOCAT_EXE-NOTFOUND)
  message(FATAL_ERROR "socat not found. Use: sudo apt install socat")
else ()
  message(STATUS "socat found in ${SOCAT_EXE}")
endif ()

setup_dep(utility $ENV{UTILITY_HOME} SUB_DIR "") 

include(test_project)
setup_tests()
find_test_srcs()
build_tests(SRCS ${SOURCES} LIBS ${PROJECT_NAME} ${BTR_LIBS} SUFFIX "-tests")

set(DOXYGEN_WARN NO)
set(DOXYGEN_PREPROCESSING YES)
build_doc(DEP ${PROJECT_NAME}-tests SUFFIX "-docs")
