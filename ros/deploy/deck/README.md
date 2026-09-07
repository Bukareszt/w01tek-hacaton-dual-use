# The Steam Deck as the panel's screen

`wojtek_deck` serves a cockpit page; this is what it takes to put that page on
a handheld and drive it from the PC. Everything here is installed by
[`../../deck.sh`](../../deck.sh) — nothing on the Deck should be hand-made.

```bash
./ros/deck.sh install        # helpers + desktop icons onto the Deck
./ros/deck.sh panel          # open the panel, pointed at this PC's simulation
./ros/deck.sh shot           # see what is on its screen
```

## What is awkward about this machine, and why each piece exists

**No sudo.** The `deck` account's password was set once in 2022 and nobody
remembers it, so there is no system service, no udev rule, no package
install. Everything runs as the user: a user-level sshd, flatpak Chrome, and
`systemd-run --user` wherever something must survive the shell that started
it.

**ssh has to be started from the couch.** The sshd is that user-level one and
it dies on every reboot, so there is a moment when no command from the PC can
reach the Deck at all — including the command that would start it. That is
what the `Deck SSH on` icon is: tap it, read the address off the notification,
and the PC can get in. `deck.sh` finds that address by itself afterwards.

**Its address moves.** DHCP, and no anchor like the robot's `10.42.0.2`. So
no address is written down in this tree; `deck.sh` looks for the one host on
the subnet answering on the ssh port and remembers it. Set `DECK_HOST` in
`ros/.env` to skip the search.

**Steam takes the controller.** While Steam runs, the browser is handed a
virtual "X-Box 360 pad" that never sends an event, and the panel shows no pad.
Close Steam (`Steam off (pad)`, or `./deck.sh steam off`) and the kernel's
hid-steam driver exposes the Deck itself as `js0`, which Chrome reads — with
the buttons in the driver's own order, which `deck.js` handles. The cost is
that Steam is also the only on-screen keyboard the Deck has.

**No keyboard means no way out of a full-screen window.** This is why the
panel window has a frame: minimise and close are reachable with a finger, and
the page's own `full` button fills the screen and gives it back, which is
what F11 would do. The `Panel STOP` and `Panel RELOAD` icons are the same
idea for the cases where the page itself is not answering.

## The pieces

| file | runs on | what it is |
| --- | --- | --- |
| `deck_link.sh` | Deck | ssh up, Steam off/on, screenshot |
| `deck_panel.sh` | Deck | opens Chrome on the panel, with the flags it needs |
| `panel_ctl.sh` | Deck | start / stop / reload, from an icon or over ssh |
| `*.desktop` | Deck | the five icons, installed onto its desktop |

`deck.sh install` also writes `~/.config/wojtek/panel-url` on the Deck, which
is where the panel icon learns which machine is serving the page. It is
written at install time rather than committed, for the same reason as the
address.

## Chrome's flags, since they are not obvious

`--enable-features=Vulkan,WebGPU --ignore-gpu-blocklist --enable-unsafe-webgpu`
— all three. With only the last one the detector lands in SwiftShader and
burns six cores; with none of them WebGPU is absent and the panel falls back
to the CPU at about 117 ms a frame against 36 on the GPU.

`--unsafely-treat-insecure-origin-as-secure` — the page is served over plain
http from the robot, which has no certificate and no internet to get one.

`--test-type` — only silences the yellow bar about the flag above.
