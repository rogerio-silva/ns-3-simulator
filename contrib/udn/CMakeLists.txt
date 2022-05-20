#/*
#*   Copyright (c) 2022 Universidade Federal de Goiás - UFG
#*
#*   This program is free software; you can redistribute it and/or modify
#*   it under the terms of the GNU General Public License version 2 as
#*   published by the Free Software Foundation;
#*
#*   This program is distributed in the hope that it will be useful,
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#*   GNU General Public License for more details.
#*
#*   You should have received a copy of the GNU General Public License
#*   along with this program; if not, write to the Free Software
#*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#*
#*   Author Rogério Sousa rogeriosousa@discente.ufg.br
#*/

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
    
