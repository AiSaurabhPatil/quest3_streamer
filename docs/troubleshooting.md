# Troubleshooting

Common issues and their solutions for TeleSim — covering connection, calibration, Isaac Sim, recording, and performance.

---

## Connection Issues

### 404 Error on Quest Browser

**Symptoms**: "File not found" or 404 error when accessing the WebXR page.

**Cause**: Incorrect URL — the `web/` subdirectory prefix is required.

**Solution**: Use the full path:
```
https://<WORKSTATION_LAN_IP>:8000/web/webxr_streamer.html
```

---

### `ERR_CERT_COMMON_NAME_INVALID` on Quest Browser

**Symptoms**: Quest browser shows a security error and refuses to load the page.

**Cause**: The SSL certificate was generated without the workstation's current LAN IP as a Subject
Alternative Name (SAN). This happens when the IP changes (e.g., after reconnecting to WiFi) or
when the cert was generated on a different machine.

**Solution**: Regenerate the certificate with the correct SAN:

```bash
rm -f certs/cert.pem certs/key.pem
bash scripts/generate_cert.sh    # auto-detects your LAN IP
```

Then restart the HTTPS server and re-navigate on the Quest.

---

### WebSocket Disconnected

**Symptoms**: "WebSocket: Disconnected" shown in the WebXR app.

**Possible Causes & Solutions**:

| Cause | Solution |
|-------|----------|
| Bridge not running | Run `./scripts/run_lan_teleop.sh --robot acone` or start the bridge manually |
| Wrong workstation IP | Confirm IP with `hostname -I | awk '{print $1}'` |
| Firewall blocking ports | Allow ports 8000 and 9999 |
| Quest and PC on different WiFi networks | Ensure both are on the same SSID |

**Debug Steps**:

1. Check if bridge is running:
   ```bash
   ps aux | grep webxr_bridge
   ```

2. Test the WebSocket port:
   ```bash
   curl -v https://localhost:9999 --insecure
   ```

3. Check and open firewall:
   ```bash
   sudo ufw status
   sudo ufw allow 8000
   sudo ufw allow 9999
   ```

---

### "WebXR Not Available"

**Cause**: WebXR requires a **Secure Context** (HTTPS). You cannot use WebXR over plain HTTP
on remote IPs.

**Solution**:
- Always use `https://` URLs
- Generate certificates: `bash scripts/generate_cert.sh`
- Accept the certificate warning in the Quest browser (click Advanced → Proceed)

---

### Certificate Warning Won't Go Away

**Cause**: Self-signed certificates are not trusted by browsers by default — this is expected.

**Solution**:
1. Click **Advanced** in the browser warning dialog
2. Click **Proceed to `<IP>` (unsafe)**

This is safe for local development with self-signed certificates.

---

## Calibration Issues

### Calibration Never Completes

**Symptoms**: Terminal shows `[Calibration] Left: Calibrating (X/30)` but never reaches 30.

**Possible Causes & Solutions**:

| Cause | Solution |
|-------|----------|
| Not holding controllers steady | Keep both controllers very still for 1–2 seconds |
| Quest not connected | Verify "WebSocket: Connected" shows in the Quest app |
| Moving during calibration | Wait until `CALIBRATION COMPLETE` before moving |

**Debug**: Watch the terminal for the calibration count:
```
[Calibration] Left: Calibrating (15/30) | Right: Calibrating (20/30)
```

---

### "No Quest controller data received"

**Symptoms**: Terminal shows "Waiting for Quest controller data..."

**Debug Steps**:

1. Check if ROS topics exist:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 topic list | grep quest
   ```

2. Check if data is flowing:
   ```bash
   ros2 topic hz /quest/right_hand/pose
   ```
   Should show ~60–90 Hz if working.

3. Verify the WebXR session is active on the Quest (tap "Start AR Session").
4. Verify the URL uses `https://` (not `http://`).

---

## Isaac Sim Issues

### Isaac Sim Won't Start / Exits Immediately

**Common Causes**:

1. **Wrong Isaac Sim path** — edit `config/config.yaml`:
   ```yaml
   paths:
     isaac_sim: "/correct/absolute/path/to/isaac_sim"
   ```

2. **Wrong `active_robot`** — set to one of `openarm`, `acone`, `ffw_bg2`:
   ```yaml
   active_robot: acone
   ```

3. **Missing USD file** — verify the path in `config/robots/<robot>.yaml` exists under `robot_configs/`.

4. **System ROS sourced before Isaac Sim** — do NOT source `/opt/ros/jazzy/setup.bash` before
   running Isaac Sim scripts. Isaac Sim uses its own bundled ROS 2 Jazzy.

---

### Robot Moves Erratically / Jitters

**Solutions**:

1. **Increase smoothing** in `config/robots/<robot>.yaml`:
   ```yaml
   teleop:
     smoothing:
       position_tau_s: 0.15   # increase from 0.10
       orientation_tau_s: 0.15
   ```

2. **Recalibrate**: Restart the script and perform calibration again with controllers held very still.

3. **Check controller tracking**: Ensure Quest controllers are visible to the headset cameras (not
   occluded by your body).

4. **Enable pose prediction** in `config/config.yaml`:
   ```yaml
   transport:
     enable_prediction: true
     prediction_horizon_ms: 40  # tune to your age_p50 value
   ```

---

### IK Failures / Arm Not Moving

**Symptoms**: Terminal shows `IK failed for target: [x, y, z]`.

**Cause**: The target position is outside the robot's reachable workspace.

**Solutions**:

| Action | Effect |
|--------|--------|
| Move hands closer to body | Keeps targets in reachable workspace |
| Lower arms | Prevents reaching above the workspace boundary |
| Avoid crossing arms | Prevents collision configurations |
| Recalibrate from a more central pose | Gives more workspace in all directions |

The robot holds its last successful joint position when IK fails. Adjust your hand position and
the arm will resume tracking.

---

### Control Loop Rate Below Target

**Symptoms**: `[Control] loop=42Hz` but `target_control_rate_hz: 60`.

**Solutions**:

1. Increase `render_every_n_steps` in `config/config.yaml`:
   ```yaml
   isaac:
     render_every_n_steps: 2   # render every 2nd physics step
   ```
   Use `4` or higher if your GPU is severely bottlenecked.

2. Reduce camera resolution in `config/config.yaml`:
   ```yaml
   cameras:
     resolution: [320, 240]
   ```

3. Close other GPU-intensive applications.

---

### Black / Frozen Viewport

**Solutions**:

1. Wait 30–60 seconds on first run — Isaac Sim shader compilation takes time.
2. Click in the viewport to focus it, then press `F` to frame the scene.
3. Check GPU memory with `nvidia-smi` — if VRAM is exhausted, reduce camera resolution or
   close other applications.

---

## Recording Issues

### Recording Worker Fails to Start

**Symptoms**: Dataset is not created; terminal shows recording worker errors.

**Diagnosis checklist**:

1. Verify `.venv/bin/python` exists:
   ```bash
   ls .venv/bin/python
   ```

2. Verify LeRobot is installed in the venv:
   ```bash
   source .venv/bin/activate
   python -c "import lerobot; print(lerobot.__version__)"
   ```
   If not: `uv pip install -r requirements.txt`

3. Check the `LEROBOT_RECORDING_PYTHON` environment variable if you use a custom venv:
   ```bash
   export LEROBOT_RECORDING_PYTHON=$PWD/.venv/bin/python
   ```

---

### Deferred Renderer Replays Too Fast or Too Slow

**Symptoms**: Robot motion in the rendered dataset looks sped up or slowed down compared to the
original teleop session.

**Solution**: The renderer auto-detects physics substeps from your config. Override explicitly:
```bash
./scripts/run_deferred_renderer.sh \
  local/quest3-acone \
  local/quest3-acone-rendered \
  --robot-type acone \
  --physics-substeps 2
```

The default formula is: `substeps = round(target_control_rate_hz / recording_fps)`.
For `60 Hz / 30 fps = 2 substeps` per rendered frame.

---

### Scene Not Matching Original Teleop (Domain Randomization)

**Symptoms**: Objects in the rendered camera images are in different positions than during teleop.

**Cause**: The sidecar JSON was not written correctly, or the renderer is not finding it.

**Check**: Confirm the sidecar files exist in the source dataset:
```bash
ls datasets/local/quest3-acone/meta/episodes/
# Should show: scene_ep_000000.json, scene_ep_000001.json, ...
```

If files are missing, the teleop session may have been interrupted before saving. Re-run the
teleop session and make sure to press **Left Primary** (B/A) to save each episode.

---

### Dataset Episodes Not Being Saved

**Symptoms**: After pressing the save button, no new Parquet files appear in the dataset.

**Common Causes**:

- Episode was never **started** — press **Left Secondary** (Y/X) first to start, then **Left Primary** to save.
- Scene was reset (Right Secondary) without saving — this discards the current unsaved episode.
- `recording.enabled` is not set to `true` and `--record` flag was not passed.

---

## Camera Issues

### Cameras Not Publishing / No Camera Data

**Symptoms**: `/camera/*` topics exist but no data is published, or topics don't exist.

**Debug**:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep camera
ros2 topic hz /camera/head_camera/image_raw
# Should show ~15 Hz (publish_interval_frames: 2 at 30 Hz loop)
```

**Common Causes**:

1. **Wrong USD prim path** — check `cameras` section in `config/robots/<robot>.yaml`:
   ```yaml
   cameras:
     head_camera:
       prim_path: "/World/ACone/Geometry/base_link/head_camera"
   ```
   Use Isaac Sim's Stage panel to find the correct prim path.

2. **Cameras disabled** — ensure `cameras.enabled: true` in `config/config.yaml`.

3. **Camera prims not loaded** — if the USD scene doesn't include cameras, they cannot be found.

---

## Performance Issues

### High Packet Drop Rate

**Symptoms**: `[Transport][direct] drop=5%` or higher.

**Solutions**:

1. Ensure Quest and workstation are on the **same 5 GHz WiFi band** (not 2.4 GHz).
2. Reduce distance between Quest and the WiFi access point.
3. Check for WiFi channel congestion with `sudo iwconfig` or a WiFi analyzer app.
4. Enable pose prediction to compensate for jitter:
   ```yaml
   transport:
     enable_prediction: true
     prediction_horizon_ms: 50
   ```

---

### High End-to-End Latency

**Symptoms**: `left_browser_age_ms` is consistently > 100 ms.

**Solutions**:

1. Tune prediction horizon to your measured `age_p50`:
   ```yaml
   transport:
     prediction_horizon_ms: 40   # set to your age_p50 value
   ```

2. Increase control loop rate and reduce rendering overhead:
   ```yaml
   isaac:
     target_control_rate_hz: 120
     render_every_n_steps: 4
   ```

3. Analyze a captured log for detailed percentile breakdowns:
   ```bash
   python scripts/summarize_metrics.py /path/to/logfile.txt
   ```

---

## Getting Help

If you're still stuck:

1. Capture the full terminal output (including any Python tracebacks).
2. Check ROS topic data: `ros2 topic echo /topic_name`
3. Review the metrics output for anomalies.
4. Open an issue on [GitHub](https://github.com/AiSaurabhPatil/telesim/issues) with:
   - Error messages and tracebacks
   - Steps to reproduce
   - System configuration (GPU, Isaac Sim version, OS)
   - Output of `python src/config_loader.py get --config config/config.yaml active_robot`
