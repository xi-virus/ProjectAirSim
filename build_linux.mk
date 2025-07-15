# ---------------------------------------------------------------------------------------------------------------------
#
# Copyright (C) Microsoft Corporation.  
# Copyright (c) 2025 IAMAI Consulting Corporation.
#
# MIT License. All rights reserved.
#
# Module Name:
#
#   build_linux.mk
#
# Abstract:
#
#   Main makefile for Linux builds to drive CMake/UBT commands.
#
# ---------------------------------------------------------------------------------------------------------------------

REDIRECT_OUTPUT = > /dev/null 2>&1

default:
	@echo "======================================================================="
	@echo "No target specified. Please run './build.sh [target]' using the following targets:"
	@echo
	@echo " all = Build + Test + Package everything"
	@echo " rebuild_all = Clean + Build + Test + Package everything"
	@echo " all_no_test = Build + Package everything"
	@echo " clean = Clean sim libs + Blocks build files"
	@echo
	@echo " simlibs_debug = Build + Package sim libs for Debug"
	@echo " simlibs_release = Build + Package sim libs for Release"
	@echo " test_simlibs_debug = Test sim libs for Debug"
	@echo " test_simlibs_release = Test sim libs for Release"
	@echo
	@echo " blocks_debuggame = Build Plugin + Blocks for DebugGame (uses Debug sim libs)"
	@echo " blocks_development = Build Plugin + Blocks for Development (uses Release sim libs)"
	@echo " blocks_shipping = Build Plugin + Blocks for Shipping (uses Release sim libs)"
	@echo
	@echo " package_simlibs = Package sim libs for Debug + Release"
	@echo " package_plugin = Package UE Plugin for Debug + Release"
	@echo " package_blocks_debuggame = Package stand-alone Blocks environment executable for DebugGame"
	@echo " package_blocks_development = Package stand-alone Blocks environment executable for Development"
	@echo " package_blocks_shipping = Package stand-alone Blocks environment executable for Shipping"
	@echo

.PHONY: all
all: simlibs_debug test_simlibs_debug simlibs_release test_simlibs_release package_simlibs package_plugin package_blocks_debuggame package_blocks_development package_blocks_shipping

.PHONY: rebuild_all
rebuild_all: clean simlibs_debug test_simlibs_debug simlibs_release test_simlibs_release package_simlibs package_plugin package_blocks_debuggame package_blocks_development package_blocks_shipping

.PHONY: all_no_test
all_no_test: simlibs_debug simlibs_release package_simlibs package_plugin package_blocks_debuggame package_blocks_development package_blocks_shipping
# ---------------------------------------------------------------------------------------------------------------------
#
# CMAKE integration.
#
# ---------------------------------------------------------------------------------------------------------------------

CMAKE_BUILD_DIR = build/linux64
CMAKE_CMD = cmake -G "Ninja"

CMAKE_DBG_BUILD_CMD = cmake --build $(CMAKE_BUILD_DIR)/Debug
CMAKE_REL_BUILD_CMD = cmake --build $(CMAKE_BUILD_DIR)/Release

.PHONY: config_simlibs_debug
config_simlibs_debug:
	@echo "======================================================================="
	@echo "Configuring the ProjectAirSimLibs project for Linux64-Debug..."
	mkdir -p $(CMAKE_BUILD_DIR)/Debug $(REDIRECT_OUTPUT)
	cd $(CMAKE_BUILD_DIR)/Debug && $(CMAKE_CMD) -DCMAKE_BUILD_TYPE=Debug ../../..

.PHONY: simlibs_debug
simlibs_debug: config_simlibs_debug
	@echo "======================================================================="
	@echo "Building the ProjectAirSimLibs project for Linux64-Debug..."
	$(CMAKE_DBG_BUILD_CMD)

.PHONY: config_simlibs_release
config_simlibs_release:
	@echo "======================================================================="
	@echo "Configuring the ProjectAirSimLibs project for Linux64-Release..."
	mkdir -p $(CMAKE_BUILD_DIR)/Release $(REDIRECT_OUTPUT)
	cd $(CMAKE_BUILD_DIR)/Release && $(CMAKE_CMD) -DCMAKE_BUILD_TYPE=Release ../../..

.PHONY: simlibs_release
simlibs_release: config_simlibs_release
	@echo "======================================================================="
	@echo "Building the ProjectAirSimLibs project for Linux64-Release..."
	$(CMAKE_REL_BUILD_CMD)

.PHONY: package_simlibs
package_simlibs: simlibs_debug simlibs_release
	@echo "======================================================================="
	@echo "Packaging sim libs for use in custom projects..."
	mkdir -p $(CURDIR)/packages/projectairsim_simlibs/
	rsync -a $(CURDIR)/unreal/Blocks/Plugins/ProjectAirSim/SimLibs/ \
		$(CURDIR)/packages/projectairsim_simlibs/
	@echo "Packaging completed to: $(CURDIR)/packages/projectairsim_simlibs"

.PHONY: clean
clean:
	@echo "======================================================================="
	@echo "Cleaning build files..."
	rm -fr $(CMAKE_BUILD_DIR) $(REDIRECT_OUTPUT)
	rm -fr physics/matlab_sfunc/_deps $(REDIRECT_OUTPUT)
	rm -fr physics/matlab_sfunc/message $(REDIRECT_OUTPUT)
	rm -fr packages/projectairsim_simlibs $(REDIRECT_OUTPUT)
	rm -fr unreal/Blocks/Plugins/ProjectAirSim/SimLibs $(REDIRECT_OUTPUT)
	@echo "Cleaning Blocks build folders..."
	rm -fr unreal/Blocks/Binaries $(REDIRECT_OUTPUT)
	rm -fr unreal/Blocks/Build $(REDIRECT_OUTPUT)
	rm -fr unreal/Blocks/Intermediate $(REDIRECT_OUTPUT)
	rm -fr unreal/Blocks/Saved $(REDIRECT_OUTPUT)
	rm -fr unreal/Blocks/Plugins/ProjectAirSim/Binaries $(REDIRECT_OUTPUT)
	rm -fr unreal/Blocks/Plugins/ProjectAirSim/Intermediate $(REDIRECT_OUTPUT)
	rm -fr unity/BlocksUnity/Assets/Plugins/* $(REDIRECT_OUTPUT)
	rm -fr packages/Blocks $(REDIRECT_OUTPUT)
	rm -fr packages/projectairsim_ue_plugin $(REDIRECT_OUTPUT)

# ---------------------------------------------------------------------------------------------------------------------
#
# Test integration.
#
# ---------------------------------------------------------------------------------------------------------------------

CTEST_DBG_CMD = ctest -C Debug -V -T test --no-compress-output
CTEST_REL_CMD = ctest -C Release -V -T test --no-compress-output
CMAKE_DBG_TEST_CMD = cd $(CMAKE_BUILD_DIR)/Debug && $(CTEST_DBG_CMD)
CMAKE_REL_TEST_CMD = cd $(CMAKE_BUILD_DIR)/Release && $(CTEST_REL_CMD)

.PHONY: test_simlibs_debug
test_simlibs_debug: simlibs_debug
	@echo "======================================================================="
	@echo "Testing the ProjectAirSimLibs project for Linux64-Debug..."
	$(CMAKE_DBG_TEST_CMD)

.PHONY: test_simlibs_release
test_simlibs_release: simlibs_release
	@echo "======================================================================="
	@echo "Testing the ProjectAirSimLibs project for Linux64-Release..."
	$(CMAKE_REL_TEST_CMD)

# ---------------------------------------------------------------------------------------------------------------------
#
# UE build/package integration.
#
# ---------------------------------------------------------------------------------------------------------------------

.PHONY: blocks_debuggame
blocks_debuggame: simlibs_debug
	@echo "======================================================================="
	@echo "Building UE plugin and game for DebugGame (with Debug sim libs) variant..."
ifndef UE_ROOT
	@echo
	@echo "ERROR: UE_ROOT environmant variable is not set. It must be set to the target \
	Unreal engine's root folder path, ex. /home/ue4/UnrealEngine-4.25.0"
else
	@echo "UE_ROOT env variable set to: $(UE_ROOT)"
	$(UE_ROOT)/Engine/Build/BatchFiles/Linux/Build.sh Blocks Linux DebugGame \
		-project=$(CURDIR)/unreal/Blocks/Blocks.uproject
	$(UE_ROOT)/Engine/Build/BatchFiles/Linux/Build.sh BlocksEditor Linux DebugGame \
		-project=$(CURDIR)/unreal/Blocks/Blocks.uproject
endif

.PHONY: blocks_development
blocks_development: simlibs_release
	@echo "======================================================================="
	@echo "Building UE plugin and game for Development (with Release sim libs) variant..."
ifndef UE_ROOT
	@echo
	@echo "ERROR: UE_ROOT environmant variable is not set. It must be set to the target \
	Unreal engine's root folder path, ex. /home/ue4/UnrealEngine-4.25.0"
else
	@echo "UE_ROOT env variable set to: $(UE_ROOT)"
	$(UE_ROOT)/Engine/Build/BatchFiles/Linux/Build.sh Blocks Linux Development \
		-project=$(CURDIR)/unreal/Blocks/Blocks.uproject
endif

.PHONY: blocks_shipping
blocks_shipping: simlibs_release
	@echo "======================================================================="
	@echo "Building UE plugin and game for Shipping (with Release sim libs) variant..."
ifndef UE_ROOT
	@echo
	@echo "ERROR: UE_ROOT environmant variable is not set. It must be set to the target \
	Unreal engine's root folder path, ex. /home/ue4/UnrealEngine-4.25.0"
else
	@echo "UE_ROOT env variable set to: $(UE_ROOT)"
	$(UE_ROOT)/Engine/Build/BatchFiles/Linux/Build.sh Blocks Linux Shipping \
		-project=$(CURDIR)/unreal/Blocks/Blocks.uproject
endif

# Cooking content uses Development Editor so packaging DebugGame needs both Debug and Release sim libs builds
.PHONY: package_blocks_debuggame
package_blocks_debuggame: simlibs_debug simlibs_release
	@echo "======================================================================="
	@echo "Building/cooking/packaging UE stand-alone game for DebugGame (with Debug sim libs) variant..."
ifndef UE_ROOT
	@echo
	@echo "ERROR: UE_ROOT environmant variable is not set. It must be set to the target \
	Unreal engine's root folder path, ex. /home/ue4/UnrealEngine-4.25.0"
else
	@echo "UE_ROOT env variable set to: $(UE_ROOT)"
	$(UE_ROOT)/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
		-project="$(CURDIR)/unreal/Blocks/Blocks.uproject" \
		-nop4 -nocompile -build -cook -compressed -pak -allmaps -stage \
		-archive -archivedirectory="$(CURDIR)/packages/Blocks/DebugGame" \
		-clientconfig=DebugGame -clean -utf8output -prereqs
endif

.PHONY: package_blocks_development
package_blocks_development: simlibs_release
	@echo "======================================================================="
	@echo "Building/cooking/packaging UE stand-alone game for Development (with Release sim libs) variant..."
ifndef UE_ROOT
	@echo
	@echo "ERROR: UE_ROOT environmant variable is not set. It must be set to the target \
	Unreal engine's root folder path, ex. /home/ue4/UnrealEngine-4.25.0"
else
	@echo "UE_ROOT env variable set to: $(UE_ROOT)"
	$(UE_ROOT)/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
		-project="$(CURDIR)/unreal/Blocks/Blocks.uproject" \
		-nop4 -nocompile -build -cook -compressed -pak -allmaps -stage \
		-archive -archivedirectory="$(CURDIR)/packages/Blocks/Development" \
		-clientconfig=Development -clean -utf8output -prereqs
endif

.PHONY: package_blocks_shipping
package_blocks_shipping: simlibs_release
	@echo "======================================================================="
	@echo "Building/cooking/packaging UE stand-alone game for Shipping (with Release sim libs) variant..."
ifndef UE_ROOT
	@echo
	@echo "ERROR: UE_ROOT environmant variable is not set. It must be set to the target \
	Unreal engine's root folder path, ex. /home/ue4/UnrealEngine-4.25.0"
else
	@echo "UE_ROOT env variable set to: $(UE_ROOT)"
	$(UE_ROOT)/Engine/Build/BatchFiles/RunUAT.sh BuildCookRun \
		-project="$(CURDIR)/unreal/Blocks/Blocks.uproject" \
		-nop4 -nocompile -build -cook -compressed -pak -allmaps -stage \
		-archive -archivedirectory="$(CURDIR)/packages/Blocks/Shipping" \
		-clientconfig=Shipping -clean -utf8output -prereqs -nodebuginfo
endif

.PHONY: package_plugin
package_plugin: blocks_debuggame blocks_development blocks_shipping
	@echo "======================================================================="
	@echo "Packaging UE plugin for use in custom environments..."
	mkdir -p $(CURDIR)/packages/projectairsim_ue_plugin/
	rsync -a --exclude 'Binaries' --exclude 'Intermediate' $(CURDIR)/unreal/Blocks/Plugins/ \
		$(CURDIR)/packages/projectairsim_ue_plugin/Plugins/
	@echo "Packaging completed to: $(CURDIR)/packages/projectairsim_ue_plugin"
