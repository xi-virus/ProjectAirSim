// Copyright (C) Microsoft Corporation.  
// Copyright (c) 2025 IAMAI Consulting Corporation.
//
// MIT License. All rights reserved.

#pragma once

#ifndef ASC_DECL
#ifdef AIRSIMCLIENT_LIB_BUILD
#define ASC_DECL __declspec(dllexport)
#else
#define ASC_DECL __declspec(dllimport)
#endif  // AIRSIMCLIENT_LIB_BUILD
#endif  // ASC_DECL
