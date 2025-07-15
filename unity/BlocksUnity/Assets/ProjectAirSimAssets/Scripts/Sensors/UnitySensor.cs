// Copyright (C) Microsoft Corporation. 
// Copyright (C) IAMAI Consulting Corporation.  

// MIT License. All rights reserved.

using System;

using UnityEngine;

namespace UnityProjectAirSim.Sensors
{
    public class UnitySensor : MonoBehaviour
    {
        public Int64 PoseUpdatedTimeStamp
        {
            get;
            set;
        }
        = 0;
    }
}