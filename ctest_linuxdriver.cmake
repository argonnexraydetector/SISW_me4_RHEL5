##
## SiSo CTest script
##
## This will run a Nightly build on menable linuxdriver
##

##
## What you need:
##
## All platforms:
## -cmake >= 2.8.2
## -Subversion command line client
## -configured kernel sources for your running kernel
##
## And of course the usual stuff like compiler and the like.
##

##
## How to setup:
##
## Write to a file my_linuxdrv_nightly.cmake:
##
## ######### begin file
## SET(SISO_SOURCE "my/path/to/the/checkout/of/trunk")
## # the binary directory does not need to exist (but it's parent)
## # it will be deleted before use
## SET(SISO_LINUXDRV_BUILD "my/path/to/the/build/dir")
##
## # if you don't want to run a Nightly, but e.g. an Experimental build
## # SET(dashboard_model "Experimental")
##
## # if your "svn" executable is not in path
## # SET(SVNCommand "path/to/my/svn")
##
## # If the build directory should be deleted after submission
## # SET(SISO_BUILD_REMOVE TRUE)
##
## # This _*MUST*_ be the last command in this file!
## INCLUDE(${SISO_SOURCE}/driver/linux/ctest_linuxdriver.cmake)
## ######### end file
##
## If you already have configured a build with testcases there is a
## ctest_linuxdrv_local.cmake file generated in your build tree that has all
## the settings of your current build.
##
## Then run this script from a compiler shell with
## ctest -S my_linuxdrv_nightly.cmake -V
##

# Check for required variables.
FOREACH (req
		SISO_SOURCE
		SISO_LINUXDRV_BUILD
	)
	IF (NOT DEFINED ${req})
		MESSAGE(FATAL_ERROR "The containing script must set ${req}")
	ENDIF ()
ENDFOREACH (req)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.2)

IF (NOT DEFINED SVNCommand)
	SET(SVNCommand svn)
ENDIF()
SET(UpdateCommand SVNCommand)

SET(CTEST_SOURCE_DIRECTORY ${SISO_SOURCE}/driver/linux)
SET(CTEST_BINARY_DIRECTORY ${SISO_LINUXDRV_BUILD})

# Select the model (Nightly, Experimental, Continuous).
IF (NOT DEFINED dashboard_model)
	SET(dashboard_model Nightly)
ENDIF()
IF (NOT "${dashboard_model}" MATCHES "^(Nightly|Experimental|Continuous)$")
	MESSAGE(FATAL_ERROR "dashboard_model must be Nightly, Experimental, or Continuous")
ENDIF()

SET(CTEST_CMAKE_GENERATOR "Unix Makefiles")

# set the site name
EXECUTE_PROCESS(COMMAND hostname
		OUTPUT_VARIABLE _MACHINE_NAME)
STRING(STRIP ${_MACHINE_NAME} CTEST_SITE)

# set the build name
IF(NOT CTEST_BUILD_NAME)
	EXECUTE_PROCESS(COMMAND uname -m
			OUTPUT_VARIABLE _MACHINE_PROCESSOR
			OUTPUT_STRIP_TRAILING_WHITESPACE)

	## FIXME: improve that once we use other compilers for test builds, too
	SET(CTEST_BUILD_NAME "gcc-${_MACHINE_PROCESSOR}")
ENDIF(NOT CTEST_BUILD_NAME)

ctest_read_custom_files(${SISO_SOURCE}/driver/linux)

ctest_empty_binary_directory(${SISO_LINUXDRV_BUILD})

ctest_start(${dashboard_model})

ctest_update(SOURCE ${SISO_SOURCE}/driver)

ctest_configure(
		BUILD ${SISO_LINUXDRV_BUILD}
		SOURCE ${SISO_SOURCE}/driver/linux
)
ctest_build()
ctest_test(${CTEST_TEST_ARGS})
ctest_submit()

IF (SISO_BUILD_REMOVE)
	FILE(REMOVE_RECURSE ${SISO_LINUXDRV_BUILD})
ENDIF (SISO_BUILD_REMOVE)
