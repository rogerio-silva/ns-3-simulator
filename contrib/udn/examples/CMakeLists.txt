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
