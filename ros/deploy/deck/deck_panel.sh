#!/bin/bash
# Open the wojtek_deck panel on this Steam Deck.
#
#   deck_panel.sh [url]
#
# Where to look for the panel, in order: the argument, then the address
# ../deck.sh wrote to ~/.config/wojtek/panel-url, then this machine. The
# address is never in this file: the repository is public, and the machine
# serving the panel changes anyway.
#
# The window has a frame, so it can be minimised and closed with a finger --
# the Deck has no keyboard, and a kiosk window cannot be left without one.
# The panel's own "full" button fills the screen and gives it back, which is
# what F11 would do.
#
# Chrome is the flatpak, running on its own profile so the Deck's ordinary
# browsing is untouched. --test-type only silences the yellow bar about the
# insecure-origin flag below it; that flag is what lets a page served over
# plain http use the APIs the panel needs. WebGPU wants all three of its
# flags: with only --enable-unsafe-webgpu it lands in SwiftShader and burns
# six cores, and with none it falls back to the CPU at about 117 ms a frame
# instead of 36 on the GPU.
URL="${1:-}"
[ -z "$URL" ] && [ -f "$HOME/.config/wojtek/panel-url" ] && URL="$(cat "$HOME/.config/wojtek/panel-url")"
URL="${URL:-http://localhost:8090/}"

ORIGIN="${URL%%/}"; ORIGIN="${ORIGIN%%\?*}"

export DISPLAY=:0 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export XAUTHORITY="$(ps -o args= -C Xorg | sed -n 's/.*-auth \([^ ]*\).*/\1/p' | head -1)"

pkill -x chrome 2>/dev/null; sleep 1
exec flatpak run com.google.Chrome --user-data-dir="$HOME/.deck-panel-profile" \
  --no-first-run --no-default-browser-check --test-type --app="$URL" \
  --enable-features=Vulkan,WebGPU --ignore-gpu-blocklist --enable-unsafe-webgpu \
  --window-position=0,0 --window-size=1280,800 \
  "--unsafely-treat-insecure-origin-as-secure=$ORIGIN"
