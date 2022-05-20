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
build_lib_example(
    NAME udn-example
    SOURCE_FILES udn-example.cc
    LIBRARIES_TO_LINK ${libudn}
)

build_lib_example(
        NAME nr5G-uavGw
        SOURCE_FILES nr5G-uavGw.cc
        LIBRARIES_TO_LINK ${libflow-monitor} ${libudn} ${libnr}
)

build_lib_example(
        NAME uavGw-loraED
        SOURCE_FILES uavGw-loraED.cc
        LIBRARIES_TO_LINK ${libudn} ${liblorawan}
)
