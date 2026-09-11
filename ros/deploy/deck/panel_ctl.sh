#!/bin/bash
# Start, stop or reload the wojtek_deck panel. The Deck has no keyboard, so
# each of these is also a desktop icon: without one there is no way to reach
# a full-screen browser window with a finger.
#
#   panel_ctl.sh start [url]
#   panel_ctl.sh stop
#   panel_ctl.sh reload
#
export DISPLAY=:0 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export XDG_RUNTIME_DIR=/run/user/1000
export XAUTHORITY="$(ps -o args= -C Xorg | sed -n 's/.*-auth \([^ ]*\).*/\1/p' | head -1)"

case "${1:-}" in
  start)
    # Chrome opens no window at all unless it is started from inside the
    # user's own session -- from anywhere else the flatpak portal answers
    # "Could not find requesting pid" and nothing appears. systemd-run --user
    # is what puts it back in the session.
    #
    # A unit that is still loaded refuses to start again, so the old one goes
    # first whether or not anything is on screen.
    systemctl --user stop deck-panel 2>/dev/null
    sleep 1
    systemd-run --user --collect --unit=deck-panel \
      --setenv=DISPLAY=:0 --setenv=XAUTHORITY="$XAUTHORITY" \
      "$HOME/deck_panel.sh" ${2:+"$2"}
    ;;
  stop)
    systemctl --user stop deck-panel 2>/dev/null
    # pkill -f would match this script's own command line over ssh and kill
    # the shell running it; -x matches the process name only.
    pkill -x chrome 2>/dev/null
    ;;
  reload)
    W="$(xdotool search --class chrome | tail -1)"
    [ -n "$W" ] && xdotool windowactivate "$W" && sleep 0.5 && xdotool key --window "$W" F5
    ;;
  *)
    sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
