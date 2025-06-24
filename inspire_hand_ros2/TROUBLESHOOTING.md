# TeleVision Inspire Bridge Troubleshooting Guide

## Issues Fixed

### 1. SSL/TLS Connection Errors
**Problem**: Repeated `BadStatusLine` errors with SSL handshake data
**Solution**: Fixed SSL configuration mismatch. When `ngrok=True`, the server runs without SSL certificates, so clients should use HTTP, not HTTPS.

### 2. Hardcoded IP Address
**Problem**: WebRTC URL was hardcoded to `https://10.60.72.189:8080/offer`
**Solution**: Made the WebRTC URL dynamic using the local IP address.

### 3. ROS2 Double Shutdown
**Problem**: RCLError due to double shutdown attempt
**Solution**: Fixed cleanup logic to prevent double shutdown.

### 4. Network Accessibility
**Problem**: Server running on `0.0.0.0` but not accessible from external devices
**Solution**: Added local IP detection and better logging.

### 5. VR Unsupported Error
**Problem**: WebXR requires HTTPS for security reasons, but we were using HTTP
**Solution**: Created SSL certificates and HTTPS version of the bridge.

## Current Status

- **Local IP**: `10.60.72.189`
- **Firewall**: Inactive (not blocking connections)
- **External connectivity**: Working
- **Ports 8012 & 8080**: Closed (server not running)

## How to Run

### Option 1: HTTP Version (No VR/WebXR)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 television_inspire_bridge.py
```
- **Access**: `http://10.60.72.189:8012`
- **Limitation**: VR mode will show "Unsupported"

### Option 2: HTTPS Version (With VR/WebXR Support)
```bash
# First, create SSL certificates
./create_ssl_certs.sh

# Then run the HTTPS version
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 television_inspire_bridge_https.py
```
- **Access**: `https://10.60.72.189:8012`
- **Note**: Accept the security warning for the self-signed certificate
- **VR Support**: Full WebXR/VR functionality available

### 3. If Network Access Fails
```bash
# Check network status
python3 test_network.py

# Setup firewall (if needed)
sudo ./setup_firewall.sh
```

## Common Issues and Solutions

### Issue: "VR Unsupported" when trying to enter VR mode
**Cause**: WebXR requires HTTPS, but you're using HTTP
**Solution**: 
1. Use the HTTPS version: `python3 television_inspire_bridge_https.py`
2. Create SSL certificates first: `./create_ssl_certs.sh`
3. Access via `https://` not `http://`

### Issue: "Connection refused" when accessing the website
**Cause**: TeleVision server not running or wrong port
**Solution**: 
1. Make sure the bridge is running
2. Check the console output for the correct URL
3. Try `http://localhost:8012` first, then the network IP

### Issue: SSL/TLS errors in console
**Cause**: Browser trying to use HTTPS on HTTP server
**Solution**: Use `http://` not `https://` in the URL (for HTTP version)

### Issue: "Security warning" in browser
**Cause**: Self-signed SSL certificate
**Solution**: Click "Advanced" and "Proceed to site" to accept the certificate

### Issue: Inspire hands not responding
**Cause**: ROS2 services not available
**Solution**:
1. Make sure inspire hand drivers are running
2. Check service names: `ros2 service list | grep inspire`
3. Verify device IDs match your setup

### Issue: Hand tracking not working
**Cause**: WebXR permissions or camera access
**Solution**:
1. Allow camera access in browser
2. Enter VR mode in the interface
3. Check browser console for errors

## Debugging Commands

```bash
# Check if services are running
ros2 service list | grep inspire

# Check network connectivity
python3 test_network.py

# Check if ports are open
netstat -tlnp | grep -E ':(8012|8080)'

# Check firewall status
sudo ufw status

# Check if SSL certificates exist
ls -la TeleVision/teleop/cert.pem TeleVision/teleop/key.pem
```

## Expected Behavior

### HTTP Version:
1. **Bridge starts**: Shows local IP and service status
2. **Web interface loads**: Hand tracking interface appears
3. **VR mode**: Shows "Unsupported" (expected)
4. **Hand movement**: Inspire hands should respond to your hand movements

### HTTPS Version:
1. **Bridge starts**: Shows local IP and service status
2. **Web interface loads**: Hand tracking interface appears
3. **VR mode**: Enter VR mode successfully
4. **Hand movement**: Inspire hands should respond to your hand movements

## File Structure

- `television_inspire_bridge.py` - HTTP version (no VR)
- `television_inspire_bridge_https.py` - HTTPS version (with VR)
- `create_ssl_certs.sh` - SSL certificate creation script
- `test_network.py` - Network connectivity test
- `setup_firewall.sh` - Firewall configuration script
- `TROUBLESHOOTING.md` - This guide

## Support

If issues persist:
1. Check the console output for error messages
2. Run `python3 test_network.py` to verify network status
3. Try accessing `http://localhost:8012` first
4. For VR support, use the HTTPS version
5. Check that all dependencies are installed 