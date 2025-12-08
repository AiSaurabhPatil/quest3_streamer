# Troubleshooting

Common issues and their solutions.

## Connection Issues

### `adb devices` list is empty
- **Cause**: USB cable not connected properly, or USB Debugging not enabled on headset.
- **Solution**: 
    1. Unplug and replug the USB cable.
    2. Put on the headset and look for a "Allow USB Debugging" prompt. Select "Always allow from this computer".
    3. Try a different USB-C cable (ensure it passes data, not just power).

### WebSocket Disconnected
- **Symptoms**: "Status: Disconnected" on the WebXR page.
- **Cause**: `adb reverse` commands not run, or `webxr_ros_bridge.py` not running.
- **Solution**:
    1. Verify `webxr_ros_bridge.py` is running and printing "Server listening...".
    2. Re-run the adb port forwarding commands:
       ```bash
       adb reverse tcp:8080 tcp:8080
       adb reverse tcp:9090 tcp:9090
       ```
    3. Refresh the WebXR page on the headset.

## WebXR Issues

### "WebXR Not Available"
- **Cause**: Trying to access the page via an IP address (e.g., `192.168.1.x`) without HTTPS. WebXR requires a Secure Context (HTTPS or localhost).
- **Solution**:
    - Always use `http://localhost:8080/...` on the headset.
    - Ensure `adb reverse` is set up so `localhost` on the headset maps to your PC.

### Black Screen in AR / Passthrough not working
- **Cause**: Browser glitch or session state issue.
- **Solution**:
    - Refresh the page.
    - Quit the Meta Quest Browser and restart it.
    - Ensure you clicked "Allow" for camera/passthrough permissions.

## Simulation Issues

### Robot moves erratically or flies away
- **Cause**: Coordinate frame mismatch or calibration issue.
- **Solution**:
    - Restart the teleop script.
    - Perform the calibration step again (hold hand steady).
    - Ensure you are using the correct units (meters vs millimeters).

### IK Failures in Isaac Sim
- **Cause**: Target position is out of reach for the robot.
- **Solution**:
    - Bring your hand closer to your body/center of workspace.
    - Check the console for "Target out of reach" warnings.
