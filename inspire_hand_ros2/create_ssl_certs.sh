#!/bin/bash
# Create SSL certificates for TeleVision local development

echo "=== Creating SSL certificates for TeleVision ==="

# Check if mkcert is installed
if ! command -v mkcert &> /dev/null; then
    echo "mkcert is not installed. Installing..."
    # For Ubuntu/Debian
    sudo apt update
    sudo apt install -y mkcert
fi

# Get local IP
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "Local IP: $LOCAL_IP"

# Install mkcert root CA
echo "Installing mkcert root CA..."
mkcert -install

# Create certificates for local development
echo "Creating certificates for localhost and $LOCAL_IP..."
cd TeleVision/teleop
mkcert -cert-file cert.pem -key-file key.pem localhost 127.0.0.1 $LOCAL_IP

echo ""
echo "=== SSL certificates created successfully! ==="
echo "Certificates are now in: TeleVision/teleop/"
echo ""
echo "To use HTTPS (required for WebXR/VR):"
echo "1. Update the bridge to use ngrok=False"
echo "2. Access: https://$LOCAL_IP:8012"
echo ""
echo "Note: You may need to accept the security warning in your browser"
echo "since this is a self-signed certificate." 