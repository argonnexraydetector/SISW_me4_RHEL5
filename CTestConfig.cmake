## This file should be placed in the root directory of your project.
## Then modify the CMakeLists.txt file in the root directory of your
## project to incorporate the testing dashboard.
## # The following are required to uses Dart and the Cdash dashboard
##   ENABLE_TESTING()
##   INCLUDE(CTest)
set(CTEST_PROJECT_NAME "LinuxDriver")
set(CTEST_NIGHTLY_START_TIME "01:00:00 GMT")

set(CTEST_DROP_METHOD "http")
set(CTEST_DROP_SITE "code.ma.silicon-software.de")
set(CTEST_DROP_LOCATION "/cdash/submit.php?project=LinuxDriver")
set(CTEST_DROP_SITE_CDASH TRUE)
SET(CTEST_UPDATE_COMMAND "svn")
