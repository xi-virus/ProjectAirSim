# Camera Streaming

## Overview

Project AirSim's camera sensors that are used as inputs to autonomous algorithms provide the full set of pixel data for every image. However, there are also use cases for using cameras to provide continuous high resolution video streams for people to watch in real time. Trying to do this by capturing and transporting every pixel of every frame is inefficient and quickly hits performance/bandwidth limits.

To enable camera streaming for live viewing in real-time as a more efficient h.264 compressed video stream sent by WebRTC protocol, Unreal provides the [PixelStreaming](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/PixelStreaming/) plugin. Project AirSim leverages PixelStreaming through camera sensors that can be configured as streaming cameras.

To view the streams, a Node.js signalling web server is run to connect to the sim server's PixelStreaming plugin and display the stream in a web browser. A web browser is used to display the stream because modern web browsers already have many optimizations for decoding h.264 WebRTC streams as a video player including hardware acceleration. This web stream player also captures the user's keyboard and mouse inputs to proxy back to the sim server, so it can function as a remote interface to the sim's viewport.

## Configuring streaming cameras

Any camera sensor capture can enable streaming its view by setting `"streaming-enabled": true` in the `capture-settings` for its image type:

```json
"capture-settings": [
  {
    "image-type": 0,
    "width": 1920,
    "height": 1080,
    "fov-degrees": 90,
    "capture-enabled": false,
    "streaming-enabled": true,
    "pixels-as-float": false,
    "compress": false,
    "target-gamma": 2.5
  }
],
```

Enabling streaming is independent of enabling capture of the raw image pixels that are sent to the client through the pub/sub or req/rep camera APIs. For example, a chase camera can be set to have streaming enabled and capture disabled for more efficient encoding/transport of the continuous feed for live viewing without needing to copy all the pixels of every frame back from GPU to CPU to pack and transport them as individual images to the client script, while a drone's downward-facing camera sensor can be set to have streaming disabled but capture enabled to only consume the images by the client script for autonomous algorithm usage.

Currently, only a single streaming camera view can be rendered and viewed at a time for any sim instance, so there is no additional cost to enable multiple streaming cameras in the scene to have them available to cycle the view through, but the FPS of the sim may vary based on the resolution of the currently active streaming camera due to its corresponding rendering load.

## Launching the sim with camera streaming on a local machine

When running on a local machine, the `streaming-enabled` cameras can be viewed through the native viewport window of the sim process, but the web viewer can also be used to see how the stream would work if the sim server was running remotely from the user/client interface.

1. Get a packaged sim binary (Unreal includes the PixelStreaming SignallingWebServer code with packaged binaries under the `{packaged folder}/Samples/PixelStreaming` folder).

    *Note: PixelStreaming does not support running in the Unreal Editor.*

2. Launch the sim binary and the signalling web server on the same machine.

    **2a. Windows**

    Start the signalling web server by running the `Start_SignallingServer.ps1` PowerShell script. The first time running, it requires an **Administrator-elevated PowerShell** because it will automatically install the prerequisites (Chocolatey, Node.js, and npm):

    ```
    > cd {Packaged binary folder}\Samples\PixelStreaming\WebServers\SignallingWebServer\platform_scripts\cmd

    > .\Start_SignallingServer.ps1 --streamerPort 8888
    ```

    Start the sim binary with command line arguments to enable the PixelStreaming server with a matching port number (e.g. 8888):

    ```
    > {Packaged binary exe} -PixelStreamingURL=ws://127.0.0.1:8888 -RenderOffScreen -Log
    ```

    **2b. Linux**

    Start the signalling web server by running the `Start_SignallingServer.sh` script. The first time running, it will prompt for sudo access because it will automatically install the prerequisites (Node.js from NodeSource APT repo, npm, express framework, cirrus-webserver STUN server, coturn TURN server, jq, vulkan-utils, pulseaudio):

    ```
    > cd {Packaged binary folder}/Samples/PixelStreaming/WebServers/SignallingWebServer/platform_scripts/bash

    > ./Start_SignallingServer.sh --streamerPort 8888
    ```

    Start the sim binary with command line arguments to enable the PixelStreaming server with a matching port number (e.g. 8888):

    ```
    {Packaged binary .sh launcher script} -PixelStreamingURL=ws://127.0.0.1:8888 -RenderOffScreen -Log
    ```

    For more command line options when launching the signalling web server, see Unreal's [Pixel Streaming Reference](https://docs.unrealengine.com/4.27/en-US/SharingAndReleasing/PixelStreaming/PixelStreamingReference/) page.

3. Open a web browser and go to 127.0.0.1 to connect to the signalling web server and start the streaming view from the sim.

4. Click on the streaming view to start capturing the mouse and keyboard inputs to be proxied to the sim server. While captured, use the `Tab` key to cycle through the streaming-enabled cameras for each robot in the scene. Press the `Esc` key to release the mouse/keyboard capturing.

## Launching the sim with camera streaming on Azure or a remote compute machine

To connect to a remote PixelStreaming view, such as running the sim server on Azure, the following is needed:

1. Ensure port 80 is open for TCP and UDP on the sim server compute machine (including through any OS firewall settings) to allow the stream viewer web browser to connect to the Signalling Web Server and proxy user inputs back to the server.

2. The Signalling Web Server launching scripts need to detect the server's public IP for the STUN server to connect the client to the server over the internet. As of UE 5.2, there are some bugs in the launching scripts that prevent the public IP from being detected correctly. To fix these bugs, manually modify the scripts as follows:

    **(Linux) Start_Common.sh**

    Change
    ```
    publicip=$(curl -s https://api.ipify.org > /dev/null)
    ```
    to
    ```
    publicip=$(curl -s https://api.ipify.org)
    ```

    **(Windows) Start_Common.ps1**

    Change
    ```
    $global:publicip = Invoke-WebRequest -Uri "https://api.ipify.org" -UseBasicParsing
    if ($global:PublicIP -ne $null -Or $global:PublicIP.length -eq 0) {
    ```
    to
    ```
    $global:PublicIP = Invoke-WebRequest -Uri "https://api.ipify.org" -UseBasicParsing
    if ($global:PublicIP -eq $null -Or $global:PublicIP.length -eq 0) {
    ```

    When launching the Signalling Web Server, you should see the public IP detected in the script's print out instead of the default 127.0.0.1 local IP.

3. Connect a web browser to the public IP address of the sim server and if everything is configured correctly, you should see the stream view player web page that is hosted by the Signalling Web Server.

---

Copyright (C) Microsoft Corporation.  
Copyright (c) 2025 IAMAI Consulting Corporation.

MIT License. All rights reserved.
