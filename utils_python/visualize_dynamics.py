#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SO3四旋翼仿真器动力学数据可视化脚本
用于分析和可视化test_dynamics程序输出的数据
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from matplotlib import rcParams

# 设置英文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']
plt.rcParams['axes.unicode_minus'] = False

# 设置图表样式
sns.set_style("whitegrid")
rcParams['figure.figsize'] = (15, 10)

def load_data(filename='dynamics_data.csv'):
    """Load dynamics data"""
    try:
        # Read CSV file without header
        data = pd.read_csv(filename, header=None, 
                          names=['time', 'height', 'yaw', 'pitch', 'roll', 
                                 'omega_x', 'omega_y', 'omega_z', 'motor_rpm'])
        print(f"Successfully loaded data with {len(data)} rows")
        print(f"Time range: {data['time'].min():.3f}s - {data['time'].max():.3f}s")
        return data
    except FileNotFoundError:
        print(f"Error: File {filename} not found")
        print("Please run first: devel/lib/so3_quadrotor_simulator/test_dynamics > dynamics_data.csv")
        return None
    except Exception as e:
        print(f"Error loading data: {e}")
        return None

def plot_height_control(data):
    """Plot height control performance"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Height response
    ax1.plot(data['time'], data['height'], 'b-', linewidth=2, label='Actual Height')
    ax1.axhline(y=0.5, color='r', linestyle='--', alpha=0.7, label='Desired Height (0.5m)')
    ax1.set_ylabel('Height (m)', fontsize=12)
    ax1.set_title('Height Control Response', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Height error
    height_error = data['height'] - 0.5
    ax2.plot(data['time'], height_error, 'g-', linewidth=2, label='Height Error')
    ax2.axhline(y=0, color='r', linestyle='--', alpha=0.7)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Height Error (m)', fontsize=12)
    ax2.set_title('Height Error', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('height_control.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_attitude_control(data):
    """Plot attitude control performance"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Euler angles
    axes[0, 0].plot(data['time'], np.rad2deg(data['yaw']), 'b-', linewidth=2, label='Yaw')
    axes[0, 0].set_ylabel('Yaw (deg)', fontsize=12)
    axes[0, 0].set_title('Yaw Response', fontsize=14, fontweight='bold')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    axes[0, 1].plot(data['time'], np.rad2deg(data['pitch']), 'g-', linewidth=2, label='Pitch')
    axes[0, 1].set_ylabel('Pitch (deg)', fontsize=12)
    axes[0, 1].set_title('Pitch Response', fontsize=14, fontweight='bold')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Angular velocity
    axes[1, 0].plot(data['time'], data['omega_x'], 'r-', linewidth=2, label='ωx')
    axes[1, 0].plot(data['time'], data['omega_y'], 'g-', linewidth=2, label='ωy')
    axes[1, 0].plot(data['time'], data['omega_z'], 'b-', linewidth=2, label='ωz')
    axes[1, 0].set_xlabel('Time (s)', fontsize=12)
    axes[1, 0].set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    axes[1, 0].set_title('Angular Velocity Response', fontsize=14, fontweight='bold')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Roll angle
    axes[1, 1].plot(data['time'], np.rad2deg(data['roll']), 'm-', linewidth=2, label='Roll')
    axes[1, 1].set_xlabel('Time (s)', fontsize=12)
    axes[1, 1].set_ylabel('Roll (deg)', fontsize=12)
    axes[1, 1].set_title('Roll Response', fontsize=14, fontweight='bold')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('attitude_control.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_motor_performance(data):
    """Plot motor performance"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Motor RPM
    ax1.plot(data['time'], data['motor_rpm'], 'purple', linewidth=2, label='Motor RPM')
    ax1.set_ylabel('RPM', fontsize=12)
    ax1.set_title('Motor RPM Response', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # RPM change rate
    rpm_diff = np.diff(data['motor_rpm'])
    time_diff = np.diff(data['time'])
    rpm_rate = rpm_diff / time_diff
    
    ax2.plot(data['time'][1:], rpm_rate, 'orange', linewidth=2, label='RPM Change Rate')
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('RPM Change Rate (RPM/s)', fontsize=12)
    ax2.set_title('Motor RPM Change Rate', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('motor_performance.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_phase_analysis(data):
    """Plot phase space analysis"""
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Height-velocity phase plot
    height_velocity = np.gradient(data['height'], data['time'])
    axes[0, 0].plot(data['height'], height_velocity, 'b-', linewidth=1, alpha=0.7)
    axes[0, 0].set_xlabel('Height (m)', fontsize=12)
    axes[0, 0].set_ylabel('Height Rate (m/s)', fontsize=12)
    axes[0, 0].set_title('Height-Velocity Phase Plot', fontsize=14, fontweight='bold')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Attitude angle phase plot
    axes[0, 1].plot(np.rad2deg(data['roll']), np.rad2deg(data['pitch']), 'g-', linewidth=1, alpha=0.7)
    axes[0, 1].set_xlabel('Roll (deg)', fontsize=12)
    axes[0, 1].set_ylabel('Pitch (deg)', fontsize=12)
    axes[0, 1].set_title('Roll-Pitch Phase Plot', fontsize=14, fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Angular velocity phase plot
    axes[1, 0].plot(data['omega_x'], data['omega_y'], 'r-', linewidth=1, alpha=0.7)
    axes[1, 0].set_xlabel('ωx (rad/s)', fontsize=12)
    axes[1, 0].set_ylabel('ωy (rad/s)', fontsize=12)
    axes[1, 0].set_title('Angular Velocity Phase Plot', fontsize=14, fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    
    # 3D trajectory
    ax3d = fig.add_subplot(2, 2, 4, projection='3d')
    ax3d.plot(data['height'], np.rad2deg(data['roll']), np.rad2deg(data['pitch']), 
              'purple', linewidth=2, alpha=0.8)
    ax3d.set_xlabel('Height (m)', fontsize=10)
    ax3d.set_ylabel('Roll (deg)', fontsize=10)
    ax3d.set_zlabel('Pitch (deg)', fontsize=10)
    ax3d.set_title('3D State Trajectory', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('phase_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def plot_comprehensive_analysis(data):
    """Plot comprehensive analysis"""
    fig = plt.figure(figsize=(20, 12))
    
    # Create subplot grid
    gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
    
    # 1. Height control
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(data['time'], data['height'], 'b-', linewidth=2, label='Actual Height')
    ax1.axhline(y=0.5, color='r', linestyle='--', alpha=0.7, label='Desired Height')
    ax1.set_ylabel('Height (m)', fontsize=12)
    ax1.set_title('Height Control Performance', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 2. Attitude angles
    ax2 = fig.add_subplot(gs[0, 2:])
    ax2.plot(data['time'], np.rad2deg(data['roll']), 'r-', linewidth=2, label='Roll')
    ax2.plot(data['time'], np.rad2deg(data['pitch']), 'g-', linewidth=2, label='Pitch')
    ax2.plot(data['time'], np.rad2deg(data['yaw']), 'b-', linewidth=2, label='Yaw')
    ax2.set_ylabel('Angle (deg)', fontsize=12)
    ax2.set_title('Attitude Response', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. Angular velocity
    ax3 = fig.add_subplot(gs[1, :2])
    ax3.plot(data['time'], data['omega_x'], 'r-', linewidth=2, label='ωx')
    ax3.plot(data['time'], data['omega_y'], 'g-', linewidth=2, label='ωy')
    ax3.plot(data['time'], data['omega_z'], 'b-', linewidth=2, label='ωz')
    ax3.set_ylabel('Angular Velocity (rad/s)', fontsize=12)
    ax3.set_title('Angular Velocity Response', fontsize=14, fontweight='bold')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. Motor RPM
    ax4 = fig.add_subplot(gs[1, 2:])
    ax4.plot(data['time'], data['motor_rpm'], 'purple', linewidth=2, label='Motor RPM')
    ax4.set_ylabel('RPM', fontsize=12)
    ax4.set_title('Motor RPM', fontsize=14, fontweight='bold')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 5. Height error statistics
    ax5 = fig.add_subplot(gs[2, 0])
    height_error = data['height'] - 0.5
    ax5.hist(height_error, bins=50, alpha=0.7, color='skyblue', edgecolor='black')
    ax5.set_xlabel('Height Error (m)', fontsize=12)
    ax5.set_ylabel('Frequency', fontsize=12)
    ax5.set_title('Height Error Distribution', fontsize=14, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    
    # 6. Attitude angle statistics
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.hist(np.rad2deg(data['roll']), bins=30, alpha=0.7, color='red', label='Roll', edgecolor='black')
    ax6.hist(np.rad2deg(data['pitch']), bins=30, alpha=0.7, color='green', label='Pitch', edgecolor='black')
    ax6.set_xlabel('Angle (deg)', fontsize=12)
    ax6.set_ylabel('Frequency', fontsize=12)
    ax6.set_title('Attitude Angle Distribution', fontsize=14, fontweight='bold')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    # 7. Phase plot
    ax7 = fig.add_subplot(gs[2, 2])
    ax7.plot(data['height'], np.gradient(data['height'], data['time']), 'b-', linewidth=1, alpha=0.7)
    ax7.set_xlabel('Height (m)', fontsize=12)
    ax7.set_ylabel('Height Rate (m/s)', fontsize=12)
    ax7.set_title('Height Phase Plot', fontsize=14, fontweight='bold')
    ax7.grid(True, alpha=0.3)
    
    # 8. Performance metrics
    ax8 = fig.add_subplot(gs[2, 3])
    ax8.axis('off')
    
    # Calculate performance metrics
    height_error = data['height'] - 0.5
    rmse_height = np.sqrt(np.mean(height_error**2))
    max_error = np.max(np.abs(height_error))
    settling_time = data['time'][np.where(np.abs(height_error) < 0.05)[0][0]] if len(np.where(np.abs(height_error) < 0.05)[0]) > 0 else np.inf
    
    # Display performance metrics
    performance_text = f"""
Performance Metrics Analysis:
    
Height Control:
• RMSE: {rmse_height:.4f} m
• Max Error: {max_error:.4f} m
• Settling Time: {settling_time:.3f} s

Attitude Control:
• Roll Std Dev: {np.std(np.rad2deg(data['roll'])):.3f}°
• Pitch Std Dev: {np.std(np.rad2deg(data['pitch'])):.3f}°
• Yaw Std Dev: {np.std(np.rad2deg(data['yaw'])):.3f}°

Motor Performance:
• Avg RPM: {np.mean(data['motor_rpm']):.0f} RPM
• RPM Range: {np.max(data['motor_rpm']) - np.min(data['motor_rpm']):.0f} RPM
"""
    
    ax8.text(0.1, 0.9, performance_text, transform=ax8.transAxes, fontsize=11,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
    
    plt.tight_layout()
    plt.savefig('comprehensive_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()

def print_statistics(data):
    """Print statistics"""
    print("\n" + "="*60)
    print("Dynamics Test Results Statistics")
    print("="*60)
    
    # Height control statistics
    height_error = data['height'] - 0.5
    print(f"Height Control Performance:")
    print(f"  • Desired Height: 0.500 m")
    print(f"  • Average Height: {np.mean(data['height']):.4f} m")
    print(f"  • Height Std Dev: {np.std(data['height']):.4f} m")
    print(f"  • Max Error: {np.max(np.abs(height_error)):.4f} m")
    print(f"  • RMSE: {np.sqrt(np.mean(height_error**2)):.4f} m")
    
    # Attitude control statistics
    print(f"\nAttitude Control Performance:")
    print(f"  • Roll Std Dev: {np.std(np.rad2deg(data['roll'])):.3f}°")
    print(f"  • Pitch Std Dev: {np.std(np.rad2deg(data['pitch'])):.3f}°")
    print(f"  • Yaw Std Dev: {np.std(np.rad2deg(data['yaw'])):.3f}°")
    print(f"  • Angular Velocity Std Dev: {np.std(data['omega_x']):.4f}, {np.std(data['omega_y']):.4f}, {np.std(data['omega_z']):.4f} rad/s")
    
    # Motor performance statistics
    print(f"\nMotor Performance:")
    print(f"  • Average RPM: {np.mean(data['motor_rpm']):.0f} RPM")
    print(f"  • RPM Range: {np.min(data['motor_rpm']):.0f} - {np.max(data['motor_rpm']):.0f} RPM")
    print(f"  • RPM Std Dev: {np.std(data['motor_rpm']):.1f} RPM")
    
    # Time statistics
    print(f"\nTime Statistics:")
    print(f"  • Simulation Duration: {data['time'].max():.3f} s")
    print(f"  • Data Points: {len(data)}")
    print(f"  • Sampling Frequency: {len(data)/data['time'].max():.1f} Hz")

def main():
    """Main function"""
    print("SO3 Quadrotor Simulator Dynamics Data Visualization")
    print("="*50)
    
    # Load data
    data = load_data()
    if data is None:
        return
    
    # Print statistics
    print_statistics(data)
    
    # Generate various plots
    print("\nGenerating visualization plots...")
    
    # 1. Height control analysis
    print("1. Generating height control analysis plot...")
    plot_height_control(data)
    
    # 2. Attitude control analysis
    print("2. Generating attitude control analysis plot...")
    plot_attitude_control(data)
    
    # 3. Motor performance analysis
    print("3. Generating motor performance analysis plot...")
    plot_motor_performance(data)
    
    # 4. Phase space analysis
    print("4. Generating phase space analysis plot...")
    plot_phase_analysis(data)
    
    # 5. Comprehensive analysis
    print("5. Generating comprehensive analysis plot...")
    plot_comprehensive_analysis(data)
    
    print("\nVisualization completed! Generated files:")
    print("• height_control.png - Height control performance")
    print("• attitude_control.png - Attitude control performance")
    print("• motor_performance.png - Motor performance")
    print("• phase_analysis.png - Phase space analysis")
    print("• comprehensive_analysis.png - Comprehensive analysis")

if __name__ == "__main__":
    main() 