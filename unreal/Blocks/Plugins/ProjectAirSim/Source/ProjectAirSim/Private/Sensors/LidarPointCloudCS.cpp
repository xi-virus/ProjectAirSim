// Copyright (C) Microsoft Corporation.  
// Copyright (c) 2025 IAMAI Consulting Corporation.
//
// MIT License. All rights reserved.

#include "LidarPointCloudCS.h"

IMPLEMENT_GLOBAL_SHADER(FLidarPointCloudCS,
                        "/CustomShaders/LidarPointCloudCS.usf",
                        "MainComputeShader", SF_Compute);