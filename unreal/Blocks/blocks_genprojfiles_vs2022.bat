REM Copyright (C) Microsoft Corporation. 
REM Copyright (C) IAMAI  Consulting Corporation.  

REM MIT License.

@echo off
if "%UE_ROOT%" == "" (
  echo:
  echo:ERROR: UE_ROOT environmant variable is not set. It must be set to the target ^
Unreal engine's root folder path, ex. C:\Program Files\Epic Games\UE_5.0
) else (
  REM Generate Visual Studio 2022 UE project Block.sln solution file
  echo:Generating Visual Studio 2022 project files with environment variable UE_ROOT=%UE_ROOT%
  cd %~dp0
  "%UE_ROOT%\Engine\Binaries\DotNET\UnrealBuildTool\UnrealBuildTool.exe" -projectfiles -2022 -project="%~dp0\Blocks.uproject" -game
)
