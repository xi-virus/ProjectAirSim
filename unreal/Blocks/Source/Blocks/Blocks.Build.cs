// Copyright (C) Microsoft Corporation.  
// Copyright (c) 2025 IAMAI Consulting Corporation.
//
// MIT License. All rights reserved.

using UnrealBuildTool;

public class Blocks : ModuleRules
{
	public Blocks(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableExceptions = true;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore" });
	}
}
