---
title: Troubleshooting System Performance
subtitle: Reducing lag and improving system performance
layout: page
---

# Troubleshooting Drone Performance

Simulating robots and their sensors is computationally complex and can be very demanding on system resources.
We built the Docker container and tuned the simulator and visualization tools to be lightweight,
but due to the wide variety of hardware and quirks with Docker (such as not having GPU access by default),
you may experience slowdowns that impact the ability of the drone to complete the lab task. 
This section will describe how to identify issues related to system performance, and how to troubleshoot these issues

# Identifying Performance Issues

As discussed in Lab 2, quadrotors are unstable systems. 
This means that it requires constant adjustments by the onboard controllers to make sure the drone stays at the requested position.
As shown in Lab 2, when the drone starts up it should wait a few seconds, rise to a height of 3 units, and then hover as shown below.

<div class="columns is-centered">
    <div class="column is-centered is-8">
        <figure class="image is-4by3">
        <img src="../images/lab2/takeoff.gif">
        </figure>
    </div>
</div>

If the drone is not hovering, e.g. moving erratically, moving in circles around the desired location, flying off screen, etc.
then the drone controller may be having issues running fast enough to keep up with the simulation.
This is often because your machine is unable to compute the drone control commands at the set frequency.
You can imagine it as you trying to fly a drone but instead of giving it a command every 0.5s, you can only update the command every 5s.
In these cases, you should examine the below methods to improve performance.

Additionally, if the entire desktop GUI is lagging, you should consult option 2 below.

# Step 1. Adjusting the simulation rate

Inside the drone configuration file, we have added a `clockscale` parameter. 
Currently, it is set to 1, i.e., everything is computed in real-time.
By lowering it, you are slowing down the simulator's world time, i.e. running everything in slow motion and giving your machine more time to compute the commands the drone requires to fly.
If your drone is flying off the screen or looks unsteady, try lowering this value in increments of 0.1 until you find a workable solution.
You can find the file at `~/CS4501-Labs/lab2_ws/lab2_p2_ws/src/flightgoggles/flightgoggles/config/drone/drone.yaml`

```yaml
flightgoggles_uav_dynamics:
    init_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    clockscale: 1.0
    vehicle_mass: 1.0
    vehicle_inertia_xx: 0.0049
    ...
```

# Step 2. Adjusting the resolution

Because Docker does not have access to the GPU (n.b. it is possible to set up Docker with GPU access, but that is host-machine dependent and beyond the scope of this class),
it may have issues rendering your desktop at the chosen resolution.

Recall from Lab 1 that we set the default size to 800 by 600 in the [`docker-compose.yml`](https://github.com/less-lab-uva/cs4501-robotics-docker/blob/main/docker-compose.yml) file
and that you can adjust the size by editing your local `docker-compose.yml` file.
On my 1920 by 1080 display, I found that 1920 by 900 worked well, but this varies by monitor and window size.
Crucially, the larger the display, the greater the load on your CPU for rendering the desktop. 
Especially for high resolution displays, this may cause overall lag in the system, both in the simulation and in other applications such as Rviz.

To mitigate this, try lowering the resolution in side the container and then zooming in through your browser.
For example, if you have a 4k display, try running at standard resolution (such as 1920x900) and then zooming in by 200%.
Note that once you have loaded the VNC desktop inside the browser, the Ubuntu desktop will capture any key presses.
This means that the keyboard shortcuts to zoom in, e.g. `CTRL +/-` may not work; instead,
you can use the browser tab options with the cursor (for Firefox, this menu is the [hamburger button](https://en.wikipedia.org/wiki/Hamburger_button) in the corner).

```yml
  novnc:  
    image: theasp/novnc:latest
    environment:
      - DISPLAY_WIDTH=800  #EDIT 
      - DISPLAY_HEIGHT=600 #EDIT 
```

# Step 3. Talk to a TA

If you are still having issues, please reach out to a TA during lab time or office hours to discuss other options that may be unique to your system.