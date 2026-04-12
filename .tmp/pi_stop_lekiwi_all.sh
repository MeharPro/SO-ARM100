#!/bin/bash
set -euo pipefail

pkill -f 'python -m lerobot.robots.lekiwi.lekiwi_smooth_host' >/dev/null 2>&1 || true
pkill -f 'python -m lerobot.robots.lekiwi.lekiwi_host' >/dev/null 2>&1 || true
pkill -f 'python scripts/lekiwi_free_teach_host.py' >/dev/null 2>&1 || true
pkill -f 'python scripts/lekiwi_min_host.py' >/dev/null 2>&1 || true
pkill -f 'python scripts/lekiwi_resilient_host.py' >/dev/null 2>&1 || true
pkill -f '/home/pi/start_lekiwi_' >/dev/null 2>&1 || true
sleep 1
ps -ef | grep -E 'lekiwi|python' | grep -v grep || true
