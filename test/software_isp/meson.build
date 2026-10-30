# SPDX-License-Identifier: CC0-1.0

if not softisp_enabled
    subdir_done()
endif

software_isp_tests = [
    {'name': 'swstats_cpu_focus', 'sources': ['swstats_cpu_focus.cpp']},
]

foreach test : software_isp_tests
    exe = executable(test['name'], test['sources'],
                     dependencies : [libcamera_private],
                     link_with : [test_libraries],
                     include_directories : [test_includes_internal])

    test(test['name'], exe, suite : 'software_isp')
endforeach
