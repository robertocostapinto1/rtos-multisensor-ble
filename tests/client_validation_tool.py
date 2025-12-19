"""
Appendix C.1: Protocol Validation Script (client_validation_tool.py)

Description:
    Validates the "Batch-Header" protocol implementation.
    1. Protocol Efficiency: Calculates overhead vs. payload ratio.
    2. Jitter Analysis: Plots the "Sawtooth" correction pattern (Figure 37).
    3. Sample Continuity: Detects duplicate samples (Figure 39 analysis).

Usage:
    Ensure the sensor is IN MOTION during this test to validate duplicate detection.
"""

import asyncio
import struct
import time
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from bleak import BleakScanner, BleakClient

# ==============================================================================
# CONFIGURATION
# ==============================================================================
DEVICE_NAME = "BAWSimplePeri"
TEST_DURATION = 20.0  # Seconds

# UUID Definitions (Must match firmware GATT table)
UUID_BASE = "0000%s-0000-1000-8000-00805f9b34fb"
UUID_GYR_DATA  = UUID_BASE % "fff7"
UUID_GYR_FREQ  = UUID_BASE % "fff6"
UUID_GYR_START = UUID_BASE % "fff5"

# Test Target: Gyroscope @ 1666 Hz (Max Load)
CONFIG_BYTE = 0x07  # Register value for 1666 Hz
TARGET_HZ   = 1666.0

class ValidationClient:
    def __init__(self):
        self.packets = []       # Stores (arrival_time, raw_bytes)
        self.parsed_data = []   # Stores {'ts': float, 'x': int, ...}
        self.header_timestamps = [] # Stores hardware timestamps from packet headers
        self.start_time = 0
        self.end_time = 0

    def notification_handler(self, sender, data):
        """High-frequency callback. Minimal logic to avoid blocking the loop."""
        self.packets.append((time.time(), data))

    async def run_acquisition(self):
        print(f"--- SEARCHING FOR {DEVICE_NAME} ---")
        dev = await BleakScanner.find_device_by_name(DEVICE_NAME)
        if not dev:
            print("Device not found.")
            return

        async with BleakClient(dev) as client:
            print(f"Connected. MTU: {client.mtu_size}")
            
            # Subscribe
            await client.start_notify(UUID_GYR_DATA, self.notification_handler)
            
            # Configure Sensor
            print(f"Configuring Gyroscope to {TARGET_HZ} Hz...")
            await client.write_gatt_char(UUID_GYR_FREQ, bytes([CONFIG_BYTE]))
            
            # Start Streaming
            print(f"Acquiring data for {TEST_DURATION} seconds...")
            self.start_time = time.time()
            await client.write_gatt_char(UUID_GYR_START, b'\x01')
            
            await asyncio.sleep(TEST_DURATION)
            
            # Stop Streaming
            await client.write_gatt_char(UUID_GYR_START, b'\x00')
            self.end_time = time.time()
            print("Acquisition Complete.")
            
            # Flush
            await asyncio.sleep(0.5)
            await client.stop_notify(UUID_GYR_DATA)

    def process_data(self):
        print("\n--- PROCESSING & ANALYSIS ---")
        
        total_wire_bytes = 0
        header_bytes = 0
        
        # Base hardware timestamp (for unwrapping relative time)
        if not self.packets: return
        
        # --- 1. PARSING LOOP ---
        for arr_time, data in self.packets:
            total_wire_bytes += len(data)
            
            if len(data) < 4: continue
            
            # Extract Header (4 Bytes - Hardware Clock in us)
            hw_ts = struct.unpack('<I', data[0:4])[0]
            self.header_timestamps.append(hw_ts)
            header_bytes += 4
            
            # Extract Payload
            payload = data[4:]
            
            # Implicit Timing Model: Linear Projection
            # T_sample = T_header + (Index * T_period)
            period_us = 1e6 / TARGET_HZ
            
            for i in range(0, len(payload), 6):
                if i+6 > len(payload): break
                
                chunk = payload[i:i+6]
                x, y, z = struct.unpack('<hhh', chunk)
                
                # Calculate precise timestamp for this sample
                sample_idx = i // 6
                sample_ts = hw_ts + (sample_idx * period_us)
                
                self.parsed_data.append({
                    'hw_ts': sample_ts,
                    'arrival_ts': arr_time,
                    'x': x, 'y': y, 'z': z
                })

        # --- 2. GENERATE DATAFRAME ---
        df = pd.DataFrame(self.parsed_data)
        
        # --- 3. METRIC: THROUGHPUT & EFFICIENCY ---
        duration = self.end_time - self.start_time
        samples_rx = len(df)
        measured_hz = samples_rx / duration
        protocol_efficiency = ((total_wire_bytes - header_bytes) / total_wire_bytes) * 100
        
        print(f"Duration:       {duration:.2f} s")
        print(f"Total Bytes:    {total_wire_bytes} B")
        print(f"Efficiency:     {protocol_efficiency:.2f}% (Target: >98%)")
        print(f"Measured Rate:  {measured_hz:.2f} Hz")
        print(f"Clock Deviation: {((measured_hz - TARGET_HZ)/TARGET_HZ)*100:+.2f}%")

        # --- 4. METRIC: SAMPLE CONTINUITY (DUPLICATES) ---
        # Heuristic check: Consecutive identical samples indicate FIFO read overlap
        duplicates = df.duplicated(subset=['x','y','z']).sum()
        dup_rate = (duplicates / samples_rx) * 100
        print(f"Duplicates:     {duplicates} ({dup_rate:.2f}%)")
        
        # --- 5. PLOTTING (Drift & Jitter) ---
        self.plot_results(df)

    def plot_results(self, df):
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot A: Long Term Drift (Linearity Proof)
        # Normalize times to start at 0
        df['hw_time_s'] = (df['hw_ts'] - df['hw_ts'].iloc[0]) / 1e6
        df['pc_time_s'] = df['arrival_ts'] - df['arrival_ts'].iloc[0]
        
        ax1.plot(df['pc_time_s'], df['hw_time_s'], label='Reconstructed Sensor Time')
        ax1.plot([0, 20], [0, 20], 'r--', alpha=0.5, label='Ideal Reference')
        ax1.set_title("Synchronization Linearity Check")
        ax1.set_xlabel("Wall Clock (s)")
        ax1.set_ylabel("Sensor Time (s)")
        ax1.legend()
        ax1.grid(True)
        
        # Plot B: Packet Interval Jitter (The Sawtooth)
        # Calculates the delta between consecutive hardware headers
        headers = pd.Series(self.header_timestamps)
        deltas = headers.diff()
        
        ax2.plot(deltas.iloc[5:-5]) # Trim artifacts
        ax2.set_title("Packet Header Interval (Jitter Analysis)")
        ax2.set_ylabel("Microseconds")
        ax2.set_xlabel("Packet Index")
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig("validation_results.png")
        print("Plot saved to validation_results.png")

if __name__ == "__main__":
    tool = ValidationClient()
    asyncio.run(tool.run_acquisition())
    tool.process_data()