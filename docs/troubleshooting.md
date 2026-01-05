# Troubleshooting

Common issues and their solutions.

## Connection Issues

### 404 Error on Quest Browser

**Symptoms**: "File not found" or 404 error when accessing the WebXR page.

**Cause**: Incorrect URL path after folder reorganization.

**Solution**: Use the full path including `web/`:
```
https://<YOUR_PC_IP>:8000/web/webxr_streamer.html
```

---

### WebSocket Disconnected

**Symptoms**: "WebSocket: Disconnected" shown in the WebXR app.

**Possible Causes & Solutions**:

| Cause | Solution |
|-------|----------|
| ROS bridge not running | Run `./scripts/run_wireless.sh` or `python src/webxr_ros_bridge.py` |
| Wrong IP address | Enter your PC's correct IP in the WebXR form |
| Firewall blocking | Allow ports 8000 and 9090 through firewall |
| Different WiFi networks | Ensure Quest and PC are on same network |

**Debug Steps**:

1. Check if bridge is running:
   ```bash
   ps aux | grep webxr_ros_bridge
   ```

2. Test WebSocket port:
   ```bash
   curl -v https://localhost:9090 --insecure
   ```

3. Check firewall:
   ```bash
   sudo ufw status
   sudo ufw allow 8000
   sudo ufw allow 9090
   ```

---

### "WebXR Not Available"

**Cause**: WebXR requires a **Secure Context** (HTTPS). You cannot use WebXR over plain HTTP on remote IPs.

**Solution**: 
- Always use `https://` URLs
- Generate certificates: `./scripts/generate_cert.sh`
- Accept the certificate warning in Quest browser

---

### Certificate Warning Won't Go Away

**Cause**: Self-signed certificates are not trusted by browsers.

**Solution**:
1. Click "Advanced" in the warning dialog
2. Click "Proceed to <IP> (unsafe)"
3. This is expected behavior for self-signed certs

---

## Calibration Issues

### Calibration Never Completes

**Symptoms**: "Calibrating (X/30)" never reaches completion.

**Possible Causes & Solutions**:

| Cause | Solution |
|-------|----------|
| Not holding steady | Keep controllers very still for 1-2 seconds |
| Quest not streaming | Check WebSocket is connected |
| Moving during calibration | Wait until calibration completes before moving |

**Debug**: Check terminal for pose counts:
```
[Calibration] Left: Calibrating (15/30) | Right: Calibrating (20/30)
```

---

### "No Quest controller data received"

**Symptoms**: Terminal shows "Waiting for Quest controller data..."

**Cause**: Data not being received from Quest.

**Debug Steps**:

1. Check if topics exist:
   ```bash
   ros2 topic list | grep quest
   ```

2. Check if data is flowing:
   ```bash
   ros2 topic hz /quest/right_hand/pose
   ```
   Should show ~60-90 Hz if working.

3. Verify WebXR session is active on Quest

---

## Isaac Sim Issues

### Robot Moves Erratically

**Symptoms**: Robot arms jitter or move unpredictably.

**Solutions**:

1. **Increase smoothing**:
   Edit `src/isaac_openarm_teleop.py`:
   ```python
   CONFIG = {
       "smoothing": 0.95,  # Increase from 0.9
   }
   ```

2. **Recalibrate**: Restart the script and calibrate again

3. **Check tracking**: Ensure Quest controllers have good tracking (visible to headset cameras)

---

### IK Failures

**Symptoms**: "IK failed for target: [x, y, z]" in terminal.

**Cause**: Target position is unreachable for the robot.

**Solutions**:

| Action | Effect |
|--------|--------|
| Move hands closer to body | Keeps targets in reachable workspace |
| Lower arms | Prevents reaching too high |
| Avoid crossing arms | Prevents collision configurations |

The robot will hold its last successful position when IK fails.

---

### Isaac Sim Won't Start

**Symptoms**: Script exits immediately or errors on launch.

**Common Causes**:

1. **Wrong Isaac Sim path**:
   Edit `config/config.yaml`:
   ```yaml
   paths:
     isaac_sim: "/correct/path/to/isaac_sim"
   ```

2. **Sourced system ROS before Isaac Sim**:
   ```bash
   # DON'T do this before running Isaac Sim scripts:
   source /opt/ros/humble/setup.bash
   
   # Isaac Sim has its own ROS setup
   ```

3. **Missing USD file**:
   Check that `openarm_config/openarm_bimanual/openarm_bimanual.usd` exists.

---

### Black/Frozen Viewport

**Symptoms**: Isaac Sim viewport shows nothing or freezes.

**Solutions**:

1. Wait longer for initialization (30-60 seconds on first run)
2. Click in the viewport to focus it
3. Press `F` to frame the scene
4. Check GPU memory isn't exhausted

---

## Camera Issues

### Cameras Not Publishing

**Symptoms**: `/camera/*` topics exist but no data.

**Debug**:
```bash
ros2 topic hz /camera/head/image_raw
# Should show ~15 Hz
```

**Cause**: Camera prims not found in USD.

**Solution**: Ensure the USD file contains cameras at expected paths:
- `/openarm/openarm_body_link/head_camera`
- `/openarm/openarm_left_link7/left_wrist_camera`
- `/openarm/openarm_right_link7/right_wrist_camera`

---

## Performance Issues

### Low Frame Rate

**Symptoms**: Robot movement is laggy or choppy.

**Solutions**:

1. Reduce camera resolution in `src/isaac_openarm_teleop.py`:
   ```python
   CAMERA_RESOLUTION = (320, 240)  # Reduce from (480, 360)
   ```

2. Increase camera capture interval:
   ```python
   CAMERA_CAPTURE_INTERVAL = 4  # Capture every 4th frame instead of 2
   ```

3. Close other GPU-intensive applications

4. Use lower Isaac Sim render settings

---

## Getting Help

If you're still stuck:

1. Check terminal output for error messages
2. Look at ROS topic data: `ros2 topic echo /topic_name`
3. Open an issue on GitHub with:
   - Error messages
   - Steps to reproduce
   - Your system configuration
