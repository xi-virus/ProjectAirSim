// Copyright (C) Microsoft Corporation.  
// Copyright (c) 2025 IAMAI Consulting Corporation.
//
// MIT License. All rights reserved.
#pragma once

#include "Renderers/ProcMeshActor.h"
#include "assimp/scene.h"
#include <vector>

class AssimpToProcMesh {

public:
  PROJECTAIRSIM_API static void UpdateProcMesh(const aiScene* scene,
                                            AProcMeshActor * RuntimeActor);
};