// Copyright (C) Microsoft Corporation.  
// Copyright (c) 2025 IAMAI Consulting Corporation.
//
// MIT License. All rights reserved.

// dllmain.cpp : Defines the entry point for the DLL application.
#include "pch.h"
#include "src/Utils.h"

BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call,
                      LPVOID /*lpReserved*/
) {
  switch (ul_reason_for_call) {
    case DLL_PROCESS_ATTACH:
      microsoft::projectairsim::client::internal::UpdateClientVersion(hModule);
      break;

    case DLL_THREAD_ATTACH:
    case DLL_THREAD_DETACH:
    case DLL_PROCESS_DETACH:
      break;
  }
  return TRUE;
}
