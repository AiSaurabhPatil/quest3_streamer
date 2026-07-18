
## Local LAN Teleop (Quest + Workstation on same WiFi network)

Use this when Isaac Sim is running on **this workstation** and the Quest headset is
connected to the same local WiFi network. No VPN or packet forwarding needed.

### Option A — Single command (recommended)

```sh
cd /mnt/external/saurabh_data/Developement/quest3_streamer
./scripts/run_lan_teleop.sh --robot acone
```

This starts everything in one shot:
- HTTPS server (port 8000) → Quest loads the WebXR page from here
- WebSocket bridge in `direct` mode (port 9999) → receives controller data → publishes ROS topics
- Acone teleop → Isaac Sim opens with the **windowed GUI** on this workstation's monitor

> **Windowed GUI is the low-latency default for viewing on this PC's own monitor.**
> Since you view the sim directly on the workstation screen, direct rendering has
> lower visual latency than encoding to WebRTC and decoding in a client. The
> control-loop speedup comes from render throttling (`isaac.render_every_n_steps`
> in config.yaml), which renders only every Nth physics step regardless of mode.
>
> To instead stream the sim to a client on a DIFFERENT device, pass `--webrtc`
> (requires the Isaac Sim WebRTC client pointing at this host, port 49100):
> ```sh
> ./scripts/run_lan_teleop.sh --robot acone --webrtc
> ```

**On Quest browser:** `https://<WORKSTATION_LAN_IP>:8000/web/webxr_streamer.html`

> **Tip — Regenerate cert with your LAN IP as SAN (do once, before first run):**
> ```sh
> rm -f certs/cert.pem certs/key.pem
> bash scripts/generate_cert.sh    # auto-detects your LAN IP
> ```
> This prevents `ERR_CERT_COMMON_NAME_INVALID` on the Quest browser.

---

### Tuning for latency

The stack logs two metric lines every ~3 s. Watch these to tune:

```
[Transport][direct] rx=89.8Hz drop=0.2% jitter=4.1ms age_p50=31ms age_p95=74ms
[Control] loop=110.0Hz left_success=99.0% ... left_browser_age_ms=45 left_age_ms=12
```

- **`loop` Hz** — control-loop rate. Target ≥100 Hz (config: `isaac.target_control_rate_hz`).
  If it won't reach target, increase `isaac.render_every_n_steps` (render less often).
- **`age_p50` / `age_p95`** — Quest→bridge one-way transport age. Should be <50 ms on LAN.
- **`left_browser_age_ms`** — end-to-end Quest→apply latency. Target <50 ms.
- Summarize a captured run with `python scripts/summarize_metrics.py logfile.txt`.

Key knobs in `config/config.yaml`:
- `isaac.target_control_rate_hz` / `isaac.render_every_n_steps` — control vs render decoupling.
- `transport.enable_prediction` / `transport.prediction_horizon_ms` — pose prediction to cancel lag.
- `teleop.smoothing.position_tau_s` / `orientation_tau_s` — rate-invariant smoothing (~0.10 s = stable feel).
- `teleop.stale_recovery_alpha` — snappiness after a controller-data hiccup.

> **Tip — Regenerate cert with your LAN IP as SAN (do once, before first run):**
> ```sh
> rm -f certs/cert.pem certs/key.pem
> bash scripts/generate_cert.sh    # auto-detects your LAN IP
> ```
> This prevents `ERR_CERT_COMMON_NAME_INVALID` on the Quest browser.

---

### Option B — Manual (3 separate terminals)

#### Terminal 1 — HTTPS server (serves WebXR page to Quest)
```sh
cd /mnt/external/saurabh_data/Developement/quest3_streamer
source .venv/bin/activate
python3 web/https_server.py 8000 --cert certs/cert.pem --key certs/key.pem
```

#### Terminal 2 — WebSocket bridge (direct mode, publishes ROS topics locally)
```sh
cd /mnt/external/saurabh_data/Developement/quest3_streamer
source .venv/bin/activate
# Export paths to use Isaac Sim's bundled rclpy (since system ROS is not installed)
export PYTHONPATH="/home/saurabh/isaac_sim/exts/isaacsim.ros2.core/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="/home/saurabh/isaac_sim/exts/isaacsim.ros2.core/jazzy/lib:$LD_LIBRARY_PATH"
python3 -m src.launch.webxr_bridge \
  --mode direct \
  --host 0.0.0.0 \
  --port 9999 \
  --cert certs/cert.pem \
  --key certs/key.pem
```

#### Terminal 3 — Robot teleop (Isaac Sim)
```sh
cd /mnt/external/saurabh_data/Developement/quest3_streamer
bash scripts/run_ffw_bg2_teleop.sh
```

---

## Recording an episode (Local LAN)

```sh
cd /mnt/external/saurabh_data/Developement/quest3_streamer
./scripts/run_lan_teleop.sh \
  --robot acone \
  --record \
  --dataset-root datasets \
  --dataset-repo-id local/quest3-acone \
  --task "sort nuts and bolts in different bins" \
  --recording-fps 30 \
  --max-episodes 10
```
### deferred rendering of episode
```sh
cd /mnt/external/saurabh_data/Developement/quest3_streamer
./scripts/run_deferred_renderer.sh local/quest3-acone local/quest3-acone-rendered_v2 --robot-type acone
```
---

---

## Remote / VPN Teleop (legacy — separate robot PC over VPN)

Use this when Isaac Sim is running on a **separate remote PC** connected over VPN.

#### Terminal 1 (Local PC)
```sh
cd /home/saurabh/Development/quest3_streamer
source .venv/bin/activate
python3 -m src.launch.webxr_bridge \
  --mode ingress \
  --host 0.0.0.0 \
  --port 9999 \
  --cert certs/cert.pem \
  --key certs/key.pem \
  --forward-url wss://192.168.30.11:9998 \
  --forward-insecure
```

#### Terminal 2 (Local PC)
```sh
cd /home/saurabh/Development/quest3_streamer
python3 web/https_server.py 8000 --cert certs/cert.pem --key certs/key.pem
```

#### Terminal 1 (Remote PC)

```sh
cd /home/saurabh/Development/quest3_streamer
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python3 -m src.launch.webxr_bridge \
  --mode remote-receiver \
  --host 0.0.0.0 \
  --port 9998

```

#### Terminal 2 (Remote PC)

```sh
cd /home/saurabh/Development/quest3_streamer
bash scripts/run_acone_teleop.sh
```

#### Terminal 2 (Remote PC) for recording episode

```sh
cd /home/saurabh/Development/quest3_streamer
source .venv-lerobot-v21/bin/activate
./scripts/run_acone_teleop.sh \
  --record \
  --dataset-root datasets \
  --dataset-repo-id local/quest3-acone \
  --task "sort nuts and bolts in different bins" \
  --recording-fps 30 \
  --max-episodes 10
```

#### Terminal 3 (Remote PC) for visualizing the dataset
```sh
cd /home/saurabh/Development/quest3_streamer
source .venv-lerobot-v21/bin/activate
./scripts/visualize_lerobot_dataset.sh \
  --repo-id local/quest3-acone  \
  --root datasets/local/quest3-acone \
  --episode-index 0
```


#### 