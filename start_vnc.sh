#!/usr/bin/env bash
# start_vnc.sh: spin up an X server (Xvfb) + fluxbox, run x11vnc on port 5900

# 1) Start virtual framebuffer at display :1
Xvfb :1 -screen 0 1280x800x24 &
sleep 2

# 2) Start a lightweight window manager (fluxbox)
fluxbox &
sleep 1

# 3) Start x11vnc on port 5900, no password (for local dev only!)
x11vnc -display :1 -forever -nopw -listen 0.0.0.0 -rfbport 5900 &

sleep 2

# 4) Keep the container alive
exec bash
