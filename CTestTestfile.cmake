# CMake generated Testfile for 
# Source directory: /home/runner/work/terrascape/terrascape
# Build directory: /home/runner/work/terrascape/terrascape
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(BasicFunctionalityTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--basic" "--verbose")
set_tests_properties(BasicFunctionalityTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;66;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
add_test(VolumetricMeshTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--volumetric" "--verbose")
set_tests_properties(VolumetricMeshTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;67;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
add_test(PerformanceTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--performance" "--verbose")
set_tests_properties(PerformanceTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;68;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
add_test(DSPHawaiiTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--dsp" "--verbose")
set_tests_properties(DSPHawaiiTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;69;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
add_test(EdgeCaseTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--edge" "--verbose")
set_tests_properties(EdgeCaseTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;70;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
add_test(ToleranceTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--tolerance" "--verbose")
set_tests_properties(ToleranceTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;71;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
add_test(AllTests "/home/runner/work/terrascape/terrascape/bin/unified_tests" "--all" "--verbose")
set_tests_properties(AllTests PROPERTIES  _BACKTRACE_TRIPLES "/home/runner/work/terrascape/terrascape/CMakeLists.txt;72;add_test;/home/runner/work/terrascape/terrascape/CMakeLists.txt;0;")
