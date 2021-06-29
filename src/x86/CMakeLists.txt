include(x86_project)

setup_dep(utility $ENV{UTILITY_HOME} SUB_DIR "./")

find_srcs(FILTER ${MAIN_SRC})
list(APPEND LIBS ${BTR_LIBS})
build_lib(SRCS ${SOURCES} LIBS ${LIBS} INC_DIRS ${utility_INC_DIR})
