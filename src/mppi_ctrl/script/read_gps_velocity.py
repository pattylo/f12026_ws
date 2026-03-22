#!/usr/bin/env python3
"""
Simple script to read GPS coordinates and velocity using pymavlink
"""

from pymavlink import mavutil
import time
import sys

def main():
    # Connect to the autopilot
    connection_string = '/dev/ttyACM1'
    baudrate = 57600
    
    print(f"Connecting to autopilot on {connection_string}...")
    
    # Create the connection
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    
    print("Waiting for heartbeat...")
    heartbeat_received = False
    timeout = time.time() + 10  # 10 second timeout
    
    while time.time() < timeout:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg:
            heartbeat_received = True
            print(f"✓ Heartbeat received from system {msg.get_srcSystem()} component {msg.get_srcComponent()}")
            break
    
    if not heartbeat_received:
        print("ERROR: No heartbeat received. Check connection.")
        sys.exit(1)
    
    print("\nReading GPS and velocity data (Ctrl+C to stop)...\n")
    
    last_update = 0
    gps_data = {}
    vel_data = {}
    
    try:
        while True:
            msg = master.recv_match(blocking=False)
            
            if msg is None:
                time.sleep(0.01)
                continue
            
            msg_type = msg.get_type()
            
            # GPS position (primary source)
            if msg_type == 'GLOBAL_POSITION_INT':
                gps_data = {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.alt / 1000.0,
                    'rel_alt': msg.relative_alt / 1000.0
                }
                vel_data = {
                    'vx': msg.vx / 100.0,
                    'vy': msg.vy / 100.0,
                    'vz': msg.vz / 100.0,
                    'speed': ((msg.vx/100.0)**2 + (msg.vy/100.0)**2)**0.5
                }
                
                # Print update
                if time.time() - last_update > 0.1:  # Update display at 10Hz
                    print(f"\r[GPS] Lat:{gps_data['lat']:11.7f}° Lon:{gps_data['lon']:11.7f}° Alt:{gps_data['alt']:6.1f}m | "
                          f"[VEL] Spd:{vel_data['speed']:5.2f} Vx:{vel_data['vx']:5.2f} Vy:{vel_data['vy']:5.2f} Vz:{vel_data['vz']:5.2f} m/s", 
                          end='', flush=True)
                    last_update = time.time()
            
            # Raw GPS (backup)
            elif msg_type == 'GPS_RAW_INT':
                if not gps_data:
                    print(f"\r[GPS_RAW] Lat:{msg.lat/1e7:11.7f}° Lon:{msg.lon/1e7:11.7f}° Alt:{msg.alt/1000.0:6.1f}m Sats:{msg.satellites_visible}", 
                          end='', flush=True)
            
            # VFR HUD (backup velocity)
            elif msg_type == 'VFR_HUD':
                if not vel_data:
                    print(f"\r[VFR] Groundspeed:{msg.groundspeed:5.2f} m/s  Climb:{msg.climb:5.2f} m/s", 
                          end='', flush=True)
            
            # Local NED velocity
            elif msg_type == 'LOCAL_POSITION_NED':
                if not vel_data:
                    speed = (msg.vx**2 + msg.vy**2)**0.5
                    print(f"\r[LOCAL] Spd:{speed:5.2f} Vx:{msg.vx:5.2f} Vy:{msg.vy:5.2f} Vz:{msg.vz:5.2f} m/s", 
                          end='', flush=True)
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
