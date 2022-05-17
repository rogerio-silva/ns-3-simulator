set(name udn)

set(source_files
        model/udn.cc
        helper/udn-helper.cc
        )

set(header_files
        model/udn.h
        helper/udn-helper.h
        )

set(libraries_to_link
        ${libapplications}
        )

set(test_sources)

build_contrib_lib_component(
        "${name}"
        "${contrib_name}"
        "${source_files}"
        "${header_files}"
        "${libraries_to_link}"
        "${test_sources}"
        )