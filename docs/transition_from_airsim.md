# Transitioning from AirSim

## Converting an AirSim Unreal environment to Project AirSim

To convert an existing AirSim Unreal environment to Project AirSim, you can perform the following steps:

1. Delete the AirSim plugin folder in `<environment>/Plugins/AirSim`

2. Edit the environment project's `<environment>.uproject` file:

    In "Modules", delete:

        "AdditionalDependencies": [
            "AirSim"
        ]

    In "Plugins", delete:

        {
        "Name": "AirSim",
        "Enabled": true
        },

3. Add Project AirSim to the environment following **[How to add the Project AirSim Plugin to a custom Unreal environment](use_plugin.md#how-to-add-the-airsim-v-next-plugin-to-a-custom-unreal-environment)**, including the part about setting the `GameMode` to `ProjectAirSimGameMode` since `AirSimGameMode` is no longer valid.

4. On first opening of the `<environment>.uproject` file with Unreal Engine 4.25, there may be some warnings about failing to load some AirSim content such as:

        Failed to load /AirSim/StarterContent/Materials/M_Tech_Hex_Tile_Pulse.M_Tech_Hex_Tile_Pulse Referenced by StaticMeshComponent0
        Failed to load /AirSim/VehicleAdv/SUV/AutomotiveMaterials/Materials/Glass/M_Glass.M_Glass Referenced by StaticMeshComponent0
        ...

    These warnings can be cleared by simply re-saving the environment's levels after AirSim has been deleted from it.

5. Check the environment's config ini files in `<environment>/Config` to remove any AirSim-specific paths or settings.

## Major changes

1. The Sky system has been upgraded to take advantage of latest UE Sky Atmosphere and Volumetric clouds.
    - The lighting itself is the same. However, there are new API to fine tune the lighting in the scene. Checkout SetSunLightIntesity(), SetCloudShadowStrength() in the API Overview.
    - If you currently use your own scene/environment and work with ProjectAirSim by dropping it into the scene as a plugin. This might break existing TimeOfDay APIs. See the "Trasition your scene from SkySphere to SunSky" section below for dealing with this properly.


2. SetSunAngle(pitch, yaw) is discontinued and has been replaced by SetSunAngleFromDateTime(datetime, is_dst) which is more intuitive and supports more realistic sun positions depending on the location of the scene.

## Transition from SkySphere to SunSky
SunSky is the new Unreal environment that provides a more natural and integrated lighting conditions. See https://docs.unrealengine.com/4.27/en-US/BuildingWorlds/LightingAndShadows/SunSky/ for more details.

General guidelines to update your environment to SunSky:

1. First enable the "Sun Position Calculator" plugin using the Unreal Editor.
2. Delete any existing "DirectionalLight", "SkyLight", "SkyAtmosphere", "ExpoenntialHeightFog" actors from your scene.
3. Add the SunSky Actor using the "Place Actors" widget.
4. The initial scene might be very bright since the default lighting are configured to be very bright. To correct this, select the SunSky actor from the World Outliner and goto the inherited DirectionalLight actor inside the components of the SunSky.
5. There is a 'Intensity' field inside this directionalLight actor that can adjusted to your needs.
6. Make sure the ID Name of the SunSky actor  is "SunSky_2" (you can see this by hovering the mouse over the actor label in the World Outliner actor liss). The "SunSky_2" is the default unless there are other SunSky actors in your scene.
7. You can also add "Volumetric Clouds" actor to the scene to make the sky dome more realistic.

## Converting an AirSim v1 configuration to Project AirSim

*TODO Add some guidance on converting AirSim v1 configuration to Project AirSim configuration*

## Converting an AirSim v1 client code to Project AirSim

*TODO Add some guidance on converting AirSim APIs to Project AirSim APIs*

---

Copyright (C) Microsoft Corporation.  All rights reserved.
