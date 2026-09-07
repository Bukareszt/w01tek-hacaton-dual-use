# The Steam Deck as the robot's controller

The Deck shows the robot's camera and sends it commands. It runs the
`wojtek_deck` panel in a browser. The page is served by the robot, or by the
simulation on the PC.

This file is the runbook. The background is at the bottom.

## Bring it up from cold

Do these in order. Steps 1 and 2 need a finger on the Deck. The rest is
typed on the PC.

**1. Put the Deck on the desktop.** Press STEAM, then Power, then Switch to
Desktop. From the login screen that entry is missing, so log in to Steam
first.

**2. Tap `Deck SSH on`.** The icon is on the desktop. A notification gives
the Deck's address. Nothing on the PC can reach the Deck before this,
including the command that would start it. The sshd dies on every reboot, so
this step comes back every time.

**3. Tap `Steam off (pad)` if the operator will drive.** Steam keeps the
controller for itself and hands the browser a pad that sends nothing. The
notification names the pad the browser will get. Skip this step if the demo
is camera only. Closing Steam also takes away the Deck's on-screen keyboard.

**4. Put the Deck and the PC on one network.** The robot's access point is
`wojtek-link`. Check from the PC:

```bash
./ros/deck.sh where          # prints the Deck's address
```

**5. Start what serves the page.**

On the robot, camera and panel, no control stack:

```bash
ssh rpi@10.42.0.2
source /opt/ros/jazzy/setup.bash && source ~/wojtek_ws/install/setup.bash

setsid nohup taskset -c 0,1 ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -p enable_depth:=false -p enable_color:=true \
  -p rgb_camera.color_profile:="1280x720x15" -p pointcloud.enable:=false \
  -p align_depth.enable:=false -p enable_rgbd:=false -p enable_sync:=false \
  > ~/cam.log 2>&1 < /dev/null &

PYTHONPATH=$HOME/py_deps setsid nohup taskset -c 0,1 \
  ros2 run wojtek_deck deck_gateway \
  --ros-args -p port:=8090 -p assets_dir:=/home/rpi/deck_assets \
  > ~/gateway.log 2>&1 < /dev/null &
```

Cores 0 and 1 are the only ones these may use. The control loop owns 2 and 3.

In the simulation, the gateway starts by itself:

```bash
./ros/sim.sh --foxglove telemetry:=true policy:=<policy reference>
```

**6. Open the panel.**

```bash
./ros/deck.sh panel http://10.42.0.2:8090/                   # from the robot
./ros/deck.sh panel 'http://<pc>:8090/?telemetry=on'         # from the simulation
```

`./ros/deck.sh panel` with no argument points the Deck at this PC.

## Read the top band

| lamp | lit when |
|---|---|
| `LINK` | the command socket to the gateway is open |
| `BRIDGE` | telemetry is flowing, and only with `?telemetry=on` |
| `CAM n` | camera frames are arriving, n is the rate |
| `PAD` | the browser can see the controller |
| `DET` | the detector is running, with its backend and frame time |

`FULL` fills the screen and gives it back. `RELOAD` loads the page again.
The Deck has no keyboard, so these two are the only way to do either.

## Drive it

| input | action |
|---|---|
| left stick | forward, back, turn |
| right stick | strafe |
| A | arm and disarm |
| Y | stand up |
| B | lie down |
| LB, RB | stance height by 5 mm |
| D-pad up, left, right, down | paw wave, bow, sit, shake |

The buttons along the bottom of the page do the same things with a finger.

The robot stops when the sticks go quiet for half a second. The gateway
holds that timer, so a dropped wifi link stops the robot rather than
latching the last command.

Arming refuses while any joint sits more than 0.15 rad from the home pose.
The panel prints the refusal in its log.

## When it goes wrong

| what you see | what it is | what to do |
|---|---|---|
| `./ros/deck.sh` finds nothing | the Deck's sshd died with its session | tap `Deck SSH on` |
| `ERR_CONNECTION_REFUSED` | the gateway is not running | check `~/gateway.log` on the robot |
| `PAD` stays grey | Steam is running | tap `Steam off (pad)`, then press a pad button on the page |
| `PAD` stays grey with Steam closed | the browser reveals a pad only after a press | press A |
| `CAM` stays grey | the camera node died | check `~/cam.log` on the robot |
| a dash in every instrument | the bridge is not running | leave `?telemetry=on` off |
| the panel covers the whole screen | it is in full screen | tap `FULL`, which now reads `WINDOW` |
| nothing on screen responds | the page is stuck | run `./ros/deck.sh reload` |

A reboot of the robot wipes `/tmp` and stops both processes. The installed
files live in `$HOME` and survive it. Start again from step 5.

## What this machine makes awkward

**No sudo.** The `deck` account's password was set once in 2022 and nobody
remembers it. Everything runs as the user: a user-level sshd, flatpak
Chrome, and `systemd-run --user` for anything that must outlive the shell
that started it.

**No fixed address.** The Deck takes what DHCP gives it. No address is
written down in this tree. `./ros/deck.sh` looks for the one host on the
subnet answering on the ssh port. `DECK_HOST` in `ros/.env` skips that
search.

**No keyboard.** A full-screen browser window cannot be left without one,
which is why the panel window has a frame and the page carries `FULL` and
`RELOAD`. The Deck's only on-screen keyboard belongs to Steam.

**Steam owns the controller.** With Steam running the browser gets a virtual
pad that sends no events. With Steam closed the kernel exposes the Deck
itself as `js0`, with the buttons in the driver's order. `deck.js` carries
both orders and picks one when the pad connects.

## The pieces

| file | runs on | what it is |
|---|---|---|
| `../../deck.sh` | PC | login, install, panel, stop, reload, steam, shot, run |
| `deck_link.sh` | Deck | ssh up, Steam off and on, screenshot |
| `deck_panel.sh` | Deck | opens Chrome on the panel with the flags it needs |
| `panel_ctl.sh` | Deck | start, stop, reload, from an icon or over ssh |
| `*.desktop` | Deck | the five icons |

`./ros/deck.sh install` puts all of it on the Deck. It also writes
`~/.config/wojtek/panel-url`, which is where the panel icon learns which
machine serves the page.

## Chrome's flags

`--enable-features=Vulkan,WebGPU --ignore-gpu-blocklist
--enable-unsafe-webgpu` all three together. The last one alone puts the
detector in SwiftShader, which burns six cores. None of them drops the
detector to the CPU at about 117 ms a frame against 36 on the GPU.

`--unsafely-treat-insecure-origin-as-secure` lets the page use the APIs it
needs over plain http. The robot has no certificate.

`--test-type` silences Chrome's warning bar about the flag above.
