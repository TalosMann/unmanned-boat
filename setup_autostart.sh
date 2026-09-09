#!/usr/bin/env bash
# Run this ON THE PI. Installs a systemd service that runs
# test_ball_forward.py automatically on every boot — no SSH/laptop needed.
#
# Get this file onto the Pi: scp setup_autostart.sh talosgiant@talosbot.local:~/
# Then on the Pi: bash setup_autostart.sh
#
# Re-running this script updates the existing ire-chase service in place
# (e.g. it previously ran test_face_forward.py — only one script should
# own the camera/GPIO at a time, so this replaces it rather than adding
# a second service).

set -e

SERVICE_PATH=/etc/systemd/system/ire-chase.service

sudo tee "$SERVICE_PATH" > /dev/null << 'EOF'
[Unit]
Description=IRE boat red-ball-chase autonomy
After=multi-user.target

[Service]
Type=simple
User=talosgiant
WorkingDirectory=/home/talosgiant/ire
ExecStart=/usr/bin/python3 /home/talosgiant/ire/test_ball_forward.py
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable ire-chase.service

echo ""
echo "Installed. The service will now start automatically on every boot."
echo ""
echo "Useful commands:"
echo "  sudo systemctl start ire-chase    # start it right now, without rebooting"
echo "  sudo systemctl stop ire-chase     # stop it"
echo "  sudo systemctl status ire-chase   # check if it's running / see recent output"
echo "  journalctl -u ire-chase -f        # tail live logs (what used to print to your terminal)"
echo "  sudo systemctl disable ire-chase  # turn OFF auto-start on boot (keeps the service installed)"
