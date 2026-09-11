#!/bin/bash
# The Deck's side of being reachable from the PC.
#
#   deck_link.sh sshd        start the user sshd and say where to reach it
#   deck_link.sh steam-off   close Steam, so the browser can read the pad
#   deck_link.sh steam-on    bring Steam back
#   deck_link.sh shot        screenshot the screen to /tmp/deck_shot.png
#
# The sshd here is a user-level one: the deck account has a password nobody
# remembers, so there is no sudo and no system service. It dies on every
# reboot, and nothing on the PC can start it again -- which is why this is a
# desktop icon before it is a command.
export DISPLAY=:0 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
export XDG_RUNTIME_DIR=/run/user/1000
export XAUTHORITY="$(ps -o args= -C Xorg | sed -n 's/.*-auth \([^ ]*\).*/\1/p' | head -1)"

PORT="${DECK_SSH_PORT:-2222}"

# A tap on an icon shows nothing by itself, so the answer goes on the screen.
say() {
  echo "$1"
  notify-send -a "Wojtek deck" "Wojtek deck" "$1" 2>/dev/null ||
    kdialog --passivepopup "$1" 6 2>/dev/null || true
}

my_ip() { ip -4 -o addr show scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -1; }

case "${1:-}" in
  sshd)
    if pgrep -f "sshd -D -e -p $PORT" >/dev/null; then
      say "ssh already up: $USER@$(my_ip) port $PORT"
      exit 0
    fi
    systemd-run --user --collect --unit=deck-sshd \
      /usr/bin/sshd -D -e -p "$PORT" -h "$HOME/.ssh/host_key" \
      -o PidFile=/tmp/sshd.pid -o AuthorizedKeysFile=.ssh/authorized_keys >/dev/null 2>&1
    sleep 2
    if pgrep -f "sshd -D -e -p $PORT" >/dev/null; then
      say "ssh up: $USER@$(my_ip) port $PORT"
    else
      say "sshd did not start -- see: journalctl --user -u deck-sshd"
    fi
    ;;

  steam-off)
    # Steam keeps the real controller to itself and hands the browser a
    # virtual "X-Box 360 pad" that never sends an event, so the panel sees no
    # pad at all while Steam is up. With Steam closed the kernel's hid-steam
    # driver exposes the Deck itself as js0 and Chrome reads it directly.
    #
    # Sitting on its login screen Steam answers neither -shutdown nor a
    # polite signal, so it gets asked first and killed second.
    steam -shutdown >/dev/null 2>&1 &
    sleep 5
    if pgrep -x steam >/dev/null; then
      kill -9 $(pgrep -x steam) $(pgrep -x steamwebhelper) 2>/dev/null
      sleep 3
    fi
    if pgrep -x steam >/dev/null; then
      say "Steam is still running -- the browser will see no pad"
    else
      say "Steam closed. The pad is now: $(cat /sys/class/input/js0/device/name 2>/dev/null)"
    fi
    ;;

  steam-on)
    systemd-run --user --collect --unit=deck-steam /usr/bin/steam >/dev/null 2>&1
    say "Steam starting -- the browser loses the pad"
    ;;

  shot)
    spectacle -b -n -o /tmp/deck_shot.png >/dev/null 2>&1
    [ -f /tmp/deck_shot.png ] && echo /tmp/deck_shot.png
    ;;

  *)
    sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
