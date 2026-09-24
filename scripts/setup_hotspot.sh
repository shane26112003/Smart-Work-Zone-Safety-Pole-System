#!/bin/bash
# ==============================================================================
# Smart Work-Zone Safety Pole - 2.4 GHz Wi-Fi Hotspot Setup for Raspberry Pi 5
# ==============================================================================
# IMPORTANT:
# ESP32 and ESP32-S3 chips ONLY support 2.4 GHz Wi-Fi (802.11 b/g/n).
# By default, Raspberry Pi 5 creates a 5 GHz hotspot, which causes the ESP32
# to fail with "wifi sta connection error" because it cannot detect 5 GHz channels.
#
# This script configures a dedicated 2.4 GHz Access Point on Channel 6.
# ==============================================================================

set -e

SSID="SAFETY_POLE_AP"
PASSWORD="SafetyZone2026!"

echo "=================================================================="
echo " Configuring 2.4 GHz Wi-Fi Hotspot ($SSID) on Raspberry Pi 5 "
echo "=================================================================="

# Check and remove existing conflicting hotspot profile if present
if nmcli connection show "$SSID" >/dev/null 2>&1; then
    echo "[INFO] Removing existing $SSID profile..."
    sudo nmcli connection delete "$SSID"
fi

if nmcli connection show "Hotspot" >/dev/null 2>&1; then
    echo "[INFO] Removing default Hotspot profile..."
    sudo nmcli connection delete "Hotspot"
fi

echo "[INFO] Creating new 2.4 GHz hotspot on wlan0..."
sudo nmcli device wifi hotspot \
    ifname wlan0 \
    con-name "$SSID" \
    ssid "$SSID" \
    password "$PASSWORD" \
    band bg \
    channel 6

echo "[INFO] Enforcing 2.4 GHz (band bg, channel 6) and static gateway (192.168.4.1)..."
sudo nmcli connection modify "$SSID" 802-11-wireless.band bg
sudo nmcli connection modify "$SSID" 802-11-wireless.channel 6
sudo nmcli connection modify "$SSID" ipv4.addresses 192.168.4.1/24
sudo nmcli connection modify "$SSID" ipv4.method shared
sudo nmcli connection modify "$SSID" connection.autoconnect yes

echo "[INFO] Activating hotspot..."
sudo nmcli connection up "$SSID"

echo ""
echo "=================================================================="
echo " HOTSPOT SUCCESSFULLY ACTIVATED! "
echo " SSID:            $SSID"
echo " Password:        $PASSWORD"
echo " Frequency Band:  2.4 GHz (Channel 6) -> 100% Compatible with ESP32"
echo " Gateway IP:      192.168.4.1"
echo "=================================================================="
echo "You can now power on your ESP32-S3 Badge and ESP32 LiDAR module."
echo "They will connect instantly and output their assigned IP address."
echo "=================================================================="
