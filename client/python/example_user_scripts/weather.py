"""
Copyright (C) Microsoft Corporation. 
Copyright (C) IAMAI Consulting Corporation.  
MIT License.

Demonstrates using weather effect APIs. The set_external_force() API is used to add 
the physics from the impact of falling rain/snow to complement the visual effects.
"""

import asyncio

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log
from projectairsim.image_utils import ImageDisplay
from projectairsim.types import WeatherParameter


# Async main function to wrap async drone commands
async def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to display camera sub-windows
    image_display = ImageDisplay()

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        world = World(client, "scene_drone_weather.jsonc", delay_after_load_sec=2)

        # Create a Drone object to interact with a drone in the loaded sim world
        drone = Drone(client, world, "Drone1")

        # Subscribe to chase camera sensor
        chase_cam_window = "ChaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(
            drone.sensors["Chase"]["scene_camera"],
            lambda _, chase: image_display.receive(chase, chase_cam_window),
        )

        # Subscribe to the Drone's sensors with a callback to receive the sensor data
        rgb_name = "RGB-Image"
        image_display.add_image(rgb_name, subwin_idx=0)
        client.subscribe(
            drone.sensors["DownCamera"]["scene_camera"],
            lambda _, rgb: image_display.receive(rgb, rgb_name),
        )

        depth_name = "Depth-Image"
        image_display.add_image(depth_name, subwin_idx=1)
        client.subscribe(
            drone.sensors["DownCamera"]["depth_camera"],
            lambda _, depth: image_display.receive(depth, depth_name),
        )

        segmentation_name = "Segmentation-Image"
        image_display.add_image(segmentation_name, subwin_idx=2)
        client.subscribe(
            drone.sensors["DownCamera"]["segmentation_camera"],
            lambda _, segmentation: image_display.receive(
                segmentation, segmentation_name
            ),
        )

        image_display.start()

        # ------------------------------------------------------------------------------

        # Enable Weather Effects in the scene
        world.enable_weather_visual_effects()

        # Set Weather Visual Effects
        # Value is a normalized range (0-1.0) that determines the amount of
        # weather effect
        projectairsim_log().info("Starting scene with falling leaves")
        world.set_weather_visual_effects_param(
            param=WeatherParameter.MAPLE_LEAF, value=0.9
        )
        await asyncio.sleep(4)

        # ------------------------------------------------------------------------------

        # Set the drone to be ready to fly
        drone.enable_api_control()
        drone.arm()

        projectairsim_log().info("takeoff_async: starting")
        takeoff_task = await drone.takeoff_async()
        await takeoff_task
        projectairsim_log().info("takeoff_async: completed")

        # ------------------------------------------------------------------------------

        projectairsim_log().info(
            "Changing weather to heavy snow. Adding downwards external force to simulate contact with drone."
        )
        world.reset_weather_effects()
        world.enable_weather_visual_effects()
        world.set_weather_visual_effects_param(param=WeatherParameter.SNOW, value=0.9)
        drone.set_external_force([0, 0, 2])

        # ------------------------------------------------------------------------------

        # Command the drone to move up in NED coordinate system for 8 seconds
        move_up_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=-1.0, duration=8.0
        )  # schedule an async task to start the command
        projectairsim_log().info("Move-Up invoked")
        await move_up_task
        projectairsim_log().info("Move-Up completed")

        # ------------------------------------------------------------------------------

        projectairsim_log().info(
            "Changing weather to heavy rain. Adding downwards external force to simulate contact with drone."
        )
        world.reset_weather_effects()
        world.enable_weather_visual_effects()
        world.set_weather_visual_effects_param(param=WeatherParameter.RAIN, value=0.9)
        drone.set_external_force([0, 0, 1])
        await asyncio.sleep(2)

        # ------------------------------------------------------------------------------

        # Command the Drone to move down in NED coordinate system for 14 seconds
        move_down_task = await drone.move_by_velocity_async(
            v_north=0.0, v_east=0.0, v_down=0.5, duration=14.0
        )  # schedule an async task to start the command
        projectairsim_log().info("Move-Down invoked")
        await move_down_task
        projectairsim_log().info("Move-Down completed")

        # ------------------------------------------------------------------------------

        projectairsim_log().info("Adding fog to scene")
        world.set_weather_visual_effects_param(param=WeatherParameter.FOG, value=0.1)

        # ------------------------------------------------------------------------------

        projectairsim_log().info("land_async: starting")
        land_task = await drone.land_async()
        await land_task
        projectairsim_log().info("land_async: completed")

        # Shut down the drone
        drone.disarm()
        drone.disable_api_control()

    # logs exception on the console
    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # ------------------------------------------------------------------------------

        world.disable_weather_visual_effects()

        # ------------------------------------------------------------------------------

        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()

        image_display.stop()


if __name__ == "__main__":
    asyncio.run(main())  # Runner for async main function
