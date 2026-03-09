#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import os

# Try first path for launch, then for single run
try:
    data = pd.read_csv("src/seq_loop/scripts/timing_results.csv")
except FileNotFoundError:
    print("File not found in src/seq_loop/scripts/, trying local path...")
    data = pd.read_csv("timing_results.csv")

# Convert ns → microseconds
data["rtt_us"] = data["rtt_ns"] / 1000
data["send_jitter_us"] = data["send_jitter_ns"] / 1000
data["rtt_jitter_us"] = data["rtt_jitter_ns"] / 1000

# Function to plot line with min, max, average
def plot_with_stats(ax, series, title, ylabel):
    ax.plot(series, label="Data")
    avg = series.mean()
    min_val = series.min()
    max_val = series.max()
    
    ax.axhline(avg, color='green', linestyle='--', label=f'Avg: {avg:.2f}')
    ax.axhline(min_val, color='red', linestyle=':', label=f'Min: {min_val:.2f}')
    ax.axhline(max_val, color='blue', linestyle=':', label=f'Max: {max_val:.2f}')
    
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.legend()

# Line plots with statistics
plt.figure(figsize=(12,10))

plt.subplot(3,1,1)
plot_with_stats(plt.gca(), data["rtt_us"], "Round Trip Time", "RTT (µs)")

plt.subplot(3,1,2)
plot_with_stats(plt.gca(), data["send_jitter_us"], "Send Jitter", "Jitter (µs)")

plt.subplot(3,1,3)
plot_with_stats(plt.gca(), data["rtt_jitter_us"], "RTT Jitter", "Jitter (µs)")
plt.xlabel("Message index")

plt.tight_layout()
plt.show()

# Boxplots
plt.figure(figsize=(10,6))
plt.boxplot([data["rtt_us"], data["send_jitter_us"], data["rtt_jitter_us"]],
            labels=["RTT", "Send Jitter", "RTT Jitter"])
plt.title("Boxplot of Metrics")
plt.ylabel("µs")
plt.show()

# Histogram of RTT
plt.figure()
plt.hist(data["rtt_us"], bins=50)
plt.title("RTT Distribution")
plt.xlabel("RTT (µs)")
plt.ylabel("Count")
plt.show()
