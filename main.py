import pandas as pd
import matplotlib.pyplot as plt
import jatayu_uav as uav
import mission_settings as ms

# ------------------------------------------------------------------------------
# 1. SEQUENTIAL PHASE EXECUTION
# ------------------------------------------------------------------------------
import Phases.takeoff as to
import Phases.pullup as pu
import Phases.climb as cl
import Phases.cruise_climb as cc
import Phases.cruise_out as co
import Phases.loiter as lo
import Phases.cruise_in as ci
import Phases.descent as ds
import Phases.approach as ap
import Phases.flare as fl
import Phases.touchdown as td

# ------------------------------------------------------------------------------
# 2. EXPORT RAW TELEMETRY TO CSV
# ------------------------------------------------------------------------------
df = pd.DataFrame(ms.telemetry)
df['time_hr'] = df['time_s'] / 3600.0
df['time_min'] = df['time_s'] / 60.0

csv_filename = 'mission_telemetry_complete.csv'
df.to_csv(csv_filename, index=False)
print(f"--> Exported {len(df)} simulation steps to {csv_filename}")

# ------------------------------------------------------------------------------
# 3. GENERATE SEGMENTED MISSION TELEMETRY PLOTS
# ------------------------------------------------------------------------------
plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['font.size'] = 9

segments = [
    {
        'title': 'Part 1: Takeoff Ground Roll & Obstacle Clearance',
        'filename': 'plot_part1_takeoff_pullup.png',
        'phases': ['Takeoff', 'Takeoff_Rotation', 'Pull_Up'],
        'time_col': 'time_s',
        'time_label': 'Elapsed Time [Seconds]',
    },
    {
        'title': 'Part 2: Climb, Cruise-Climb & Transit-Out Recharging',
        'filename': 'plot_part2_climb_transit.png',
        'phases': ['Climb', 'Cruise_Climb', 'Cruise_Out'],
        'time_col': 'time_min',
        'time_label': 'Elapsed Time [Minutes]',
    },
    {
        # --- NEW DEDICATED TACTICAL STEALTH PLOT ---
        'title': 'Part 3: Tactical Silent Stare Protocol (Pure Electric Ingress, Stare & Egress)',
        'filename': 'plot_part3_silent_stare_protocol.png',
        'phases': ['Silent_Ingress', 'Silent_Stare', 'Silent_Egress'],
        'time_col': 'time_min',
        'time_label': 'Elapsed Time [Minutes]',
    },
    {
        # --- SUSTAINED STANDOFF ORBIT PLOT ---
        'title': 'Part 4: Sustained Standoff Loiter & Fuel Depletion',
        'filename': 'plot_part4_sustained_standoff_loiter.png',
        'phases': ['Sustained_Loiter'],
        'time_col': 'time_hr',
        'time_label': 'Elapsed Time [Hours]',
    },
    {
        'title': 'Part 5: Transit-In, Descent & Initial Approach',
        'filename': 'plot_part5_cruisein_descent_approach.png',
        'phases': ['Cruise_In', 'Descent', 'Approach_Decel', 'Approach_Glide'],
        'time_col': 'time_min',
        'time_label': 'Elapsed Time [Minutes]',
    },
    {
        'title': 'Part 6: Final Flare & Braking Rollout',
        'filename': 'plot_part6_flare_touchdown.png',
        'phases': [
            'Final_Approach',
            'Flare_Touchdown',
            'Landing_Rollout',
            'Mission_Complete',
        ],
        'time_col': 'time_s',
        'time_label': 'Elapsed Time [Seconds]',
    },
]

for seg in segments:
    sub_df = df[df['phase'].isin(seg['phases'])].copy()
    if sub_df.empty:
        print(f"Skipping empty segment: {seg['title']}")
        continue

    # Zero-base elapsed time for this specific segment
    t0 = sub_df[seg['time_col']].iloc[0]
    t = sub_df[seg['time_col']] - t0
    sub_df['t_plot'] = t

    fig, axs = plt.subplots(4, 1, figsize=(10.5, 11), sharex=True, dpi=300)
    plt.subplots_adjust(hspace=0.24)

    # Subplot A: Flight Trajectory (Altitude & True Airspeed)
    ax1 = axs[0]
    ax1.plot(
        t, sub_df['altitude_m'], color='#1F78B4', lw=1.8, label='Altitude (m)'
    )
    ax1.set_ylabel('Altitude [m MSL]', color='#1F78B4', fontweight='bold')
    ax1.grid(True, linestyle=':', alpha=0.6)

    ax1_t = ax1.twinx()
    ax1_t.plot(
        t,
        sub_df['velocity_ms'],
        color='#6A3D9A',
        lw=1.4,
        linestyle='--',
        label='Airspeed (m/s)',
    )
    ax1_t.set_ylabel('TAS [m/s]', color='#6A3D9A', fontweight='bold')
    ax1.set_title(
        f"{seg['title']} — Flight Trajectory", fontweight='bold', loc='left'
    )

    # Subplot B: Aerodynamic Power Demand & Powertrain Dispatch
    ax2 = axs[1]
    ax2.plot(
        t,
        sub_df['P_req_kW'],
        color='black',
        lw=1.2,
        linestyle=':',
        label='P_req (Aero Drag) [kW]',
    )
    ax2.plot(
        t,
        sub_df['P_ICE_kW'],
        color='#D95F02',
        lw=1.8,
        label='ICE Shaft Power [kW]',
    )
    ax2.plot(
        t,
        sub_df['P_EM_kW'],
        color='#33A02C',
        lw=1.6,
        label='Motor Power / Gen (-) [kW]',
    )
    ax2.axhline(0, color='gray', lw=0.7)
    ax2.set_ylabel('Power [kW]', fontweight='bold')
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right', framealpha=0.9)
    ax2.set_title(
        'Aerodynamic Demand vs. Powertrain Dispatch',
        fontweight='bold',
        loc='left',
    )

    # Subplot C: Energy Consumables Depletion
    ax3 = axs[2]
    ax3.plot(
        t,
        sub_df['battery_soc'] * 100.0,
        color='#008080',
        lw=1.8,
        label='Battery SoC [%]',
    )
    ax3.set_ylabel('Battery SoC [%]', color='#008080', fontweight='bold')
    ax3.grid(True, linestyle=':', alpha=0.6)

    ax3_t = ax3.twinx()
    ax3_t.plot(
        t,
        sub_df['fuel_mass_kg'],
        color='#E31A1C',
        lw=1.8,
        linestyle='--',
        label='Usable Fuel [kg]',
    )
    ax3_t.set_ylabel('Fuel Left [kg]', color='#E31A1C', fontweight='bold')
    ax3.set_title(
        'Energy Storage Depletion Profile', fontweight='bold', loc='left'
    )

    # Subplot D: Electric Machine Regimes (Discharge Assist vs. In-Flight Charge)
    ax4 = axs[3]
    ax4.plot(
        t, sub_df['P_EM_kW'], color='#33A02C', lw=1.4, label='Electric Plant Load'
    )
    ax4.fill_between(
        t,
        0,
        sub_df['P_EM_kW'],
        where=(sub_df['P_EM_kW'] > 0),
        color='#33A02C',
        alpha=0.3,
        label='Motor Drive Mode',
    )
    ax4.fill_between(
        t,
        0,
        sub_df['P_EM_kW'],
        where=(sub_df['P_EM_kW'] < 0),
        color='#D95F02',
        alpha=0.3,
        label='Generator Charge Mode',
    )
    ax4.axhline(0, color='black', lw=0.8)
    ax4.set_ylabel('EM Net Power [kW]', fontweight='bold')
    ax4.set_xlabel(seg['time_label'], fontweight='bold')
    ax4.grid(True, linestyle=':', alpha=0.6)
    ax4.legend(loc='upper right', framealpha=0.9)
    ax4.set_title(
        'Electric Machine Operating Regimes', fontweight='bold', loc='left'
    )

    # Expand Subplot A's upper Y limit slightly to give clean room for phase banners
    y_min, y_max = ax1.get_ylim()
    ax1.set_ylim(y_min, y_max * 1.18 if y_max > 0 else 100)

    # --------------------------------------------------------------------------
    # PHASE INTERVAL CENTERING, BOUNDARY LINES & STAGGERED LABELS
    # --------------------------------------------------------------------------
    sub_df['phase_block'] = (
        sub_df['phase'] != sub_df['phase'].shift(1)
    ).cumsum()
    blocks = sub_df.groupby('phase_block')

    stagger_toggle = 0
    for _, block in blocks:
        phase_name = block['phase'].iloc[0]
        t_start = block['t_plot'].iloc[0]
        t_end = block['t_plot'].iloc[-1]
        t_mid = (t_start + t_end) / 2.0

        # Draw vertical separator line at phase start (skip t=0)
        if t_start > 0:
            for ax in axs:
                ax.axvline(
                    x=t_start,
                    color='#666666',
                    linestyle='--',
                    linewidth=1.1,
                    alpha=0.75,
                )

        clean_phase_name = phase_name.replace('_', ' ')
        y_pos = 0.93 if (stagger_toggle % 2 == 0) else 0.82
        stagger_toggle += 1

        ax1.annotate(
            clean_phase_name,
            xy=(t_mid, y_pos),
            xycoords=('data', 'axes fraction'),
            ha='center',
            va='center',
            fontsize=7.5,
            fontweight='bold',
            color='#111111',
            bbox=dict(
                boxstyle='round,pad=0.25',
                facecolor='#EEEEEE',
                edgecolor='#888888',
                alpha=0.88,
                lw=0.8,
            ),
        )

    plt.tight_layout()
    plt.savefig(seg['filename'], dpi=300)
    plt.close(fig)
    print(f"--> Saved segment plot: {seg['filename']}")

print('\nAll 6 segmented telemetry plots successfully generated!')
