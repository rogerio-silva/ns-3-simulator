check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources    
        #test/udn-examples-test-suite.cc
        )
endif()    

build_lib(
    LIBNAME udn
    SOURCE_FILES model/udn.cc
                 helper/udn-helper.cc 
    HEADER_FILES model/udn.h
                 helper/udn-helper.h
    LIBRARIES_TO_LINK ${libcore}
    TEST_SOURCES test/udn-test-suite.cc
                 ${examples_as_tests_sources}
)
    
