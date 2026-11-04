import os
import math
import importlib
from contextlib import redirect_stdout
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ------------------------------------------------------------------------------
# 0. SUPPRESS INITIAL PRINTOUTS ON IMPORT
# ------------------------------------------------------------------------------
with open(os.devnull, 'w', encoding='utf-8') as fnull:
    with redirect_stdout(fnull):
        import jatayu_uav as uav
        import mission_settings as ms

PHASE_MODULE_NAMES = [
    'Phases.takeoff',
    'Phases.pullup',
    'Phases.climb',
    'Phases.cruise_climb',
    'Phases.cruise_out',
    'Phases.loiter',
    'Phases.cruise_in',
    'Phases.descent',
    'Phases.approach',
    'Phases.flare',
    'Phases.touchdown'
]

# Baseline fixed hardware anchors
P_ICE_BASE  = float(uav.P_ICE_rated)
P_EM_BASE   = float(ms.power_motor)
E_BATT_BASE = float(ms.battery_capacity)

# Sweep altitude from 1,000 m to 5,500 m (MSL)
ALTITUDE_SWEEP_M = np.linspace(1000.0, 5500.0, 10)
sweep_records = []

print("=" * 96)
print(f"{'PROJECT JATAYU 2.0 - HE: ALTITUDE SENSITIVITY SWEEP (h vs. MISSION LOITER)':^96}")
print("=" * 96)

for idx, alt_m in enumerate(ALTITUDE_SWEEP_M):
    # --------------------------------------------------------------------------
    # 1. MUTATE ALTITUDE IN MISSION SETTINGS
    # --------------------------------------------------------------------------
    ms.cruise_altitude = float(alt_m)
    ms.loiter_altitude = float(alt_m)

    # --------------------------------------------------------------------------
    # 2. RESET TELEMETRY BUFFER (Full 12-Channel Schema)
    # --------------------------------------------------------------------------
    ms.telemetry = {
        'time_s': [], 'phase': [], 'altitude_m': [], 'velocity_ms': [],
        'P_req_kW': [], 'P_ICE_kW': [], 'P_EM_kW': [], 'fuel_mass_kg': [], 'battery_soc': [],
        'xi_ICE': [], 'BSFC_g_kWh': [], 'm_dot_fuel_gps': []
    }

    # --------------------------------------------------------------------------
    # 3. EXECUTE PHASES SILENTLY & VALIDATE Phase_Success
    # --------------------------------------------------------------------------
    sim_failed = False
    failed_phase_name = None

    with open(os.devnull, 'w', encoding='utf-8') as fnull:
        with redirect_stdout(fnull):
            for mod_name in PHASE_MODULE_NAMES:
                try:
                    mod = importlib.import_module(mod_name)
                    importlib.reload(mod)

                    phase_ok = getattr(mod, 'Phase_Success', True)
                    if not phase_ok:
                        sim_failed = True
                        failed_phase_name = mod_name
                        break

                except Exception as e:
                    sim_failed = True
                    failed_phase_name = f"{mod_name} (Crash: {e})"
                    break

    if sim_failed:
        print(f"[{idx+1:02d}/{len(ALTITUDE_SWEEP_M):02d}] Alt: {alt_m:5.0f} m --> DISCARDED (Failed in {failed_phase_name})")
        continue

    # --------------------------------------------------------------------------
    # 4. EXTRACT MISSION METRICS
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)
    total_hr = df_sim['time_s'].iloc[-1] / 3600.0
    
    df_stare = df_sim[df_sim['phase'] == 'Silent_Stare']
    t_silent_min = ((df_stare['time_s'].iloc[-1] - df_stare['time_s'].iloc[0]) / 60.0) if not df_stare.empty else 0.0

    df_loiter = df_sim[df_sim['phase'] == 'Sustained_Loiter']
    t_loiter_hr = ((df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0) if not df_loiter.empty else 0.0

    df_ice = df_sim[df_sim['P_ICE_kW'] > 0.5]
    avg_bsfc = df_ice['BSFC_g_kWh'].mean() if not df_ice.empty else 0.0

    sweep_records.append({
        'Altitude_m': alt_m,
        'Altitude_ft': alt_m * 3.28084,
        'Total_Endurance_hr': total_hr,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Silent_Stare_min': t_silent_min,
        'Mean_BSFC_g_kWh': avg_bsfc
    })

    print(f"[{idx+1:02d}/{len(ALTITUDE_SWEEP_M):02d}] Alt: {alt_m:5.0f} m ({alt_m*3.28084:5.0f} ft) | "
          f"Loiter: {t_loiter_hr:5.2f} hr | Silent: {t_silent_min:5.1f} min | BSFC: {avg_bsfc:5.1f} g/kWh | Tot: {total_hr:5.2f} hr")

# ------------------------------------------------------------------------------
# 5. EXPORT DATA & DUAL-AXIS SENSITIVITY PLOT
# ------------------------------------------------------------------------------
if not sweep_records:
    print("\nError: All altitude design points failed or were skipped.")
    exit(1)

df_res = pd.DataFrame(sweep_records)
df_res.to_csv('altitude_sweep_results.csv', index=False)
print("\n--> Results saved to altitude_sweep_results.csv")

fig, ax1 = plt.subplots(figsize=(10, 6.2), dpi=300)

color_ice = '#1F78B4'    # Navy/Steel Blue
color_stare = '#33A02C'  # Forest Green

# Primary Left Axis: Sustained ICE Loiter
ax1.plot(
    df_res['Altitude_m'],
    df_res['Sustained_Loiter_hr'],
    marker='o',
    color=color_ice,
    lw=2.2,
    label='Sustained Standoff Loiter (Pure ICE) [Hours]'
)
ax1.set_xlabel('Operating Cruise & Loiter Altitude [m MSL]', fontweight='bold', fontsize=11)
ax1.set_ylabel('Sustained Standoff Loiter [Hours]', color=color_ice, fontweight='bold', fontsize=11)
ax1.tick_params(axis='y', labelcolor=color_ice)
ax1.grid(True, linestyle=':', alpha=0.5)

# Secondary Right Axis: Silent Stare Duration
ax2 = ax1.twinx()
ax2.plot(
    df_res['Altitude_m'],
    df_res['Silent_Stare_min'],
    marker='s',
    color=color_stare,
    lw=2.2,
    linestyle='--',
    label='Silent Stare on Station (Pure Electric) [Minutes]'
)
ax2.set_ylabel('Silent Stare Duration [Minutes]', color=color_stare, fontweight='bold', fontsize=11)
ax2.tick_params(axis='y', labelcolor=color_stare)

# Constraint Threshold Overlays & Shading
MIN_LOITER_HR = 12.0
MIN_STARE_MIN = 30.0

ax1.axhline(MIN_LOITER_HR, color='#E31A1C', linestyle=':', lw=1.6, label=f'Req: Min Loiter ({MIN_LOITER_HR:.0f} hr)')
ax2.axhline(MIN_STARE_MIN, color='#6A3D9A', linestyle=':', lw=1.6, label=f'Req: Min Stare ({MIN_STARE_MIN:.0f} min)')

y1_max = max(df_res['Sustained_Loiter_hr'].max() * 1.08, MIN_LOITER_HR + 4.0)
ax1.set_ylim(bottom=0.0, top=y1_max)
ax1.axhspan(MIN_LOITER_HR, y1_max, color='#33A02C', alpha=0.07, label='Compliant Standoff Loiter Region (≥12 hr)')

y2_max = max(df_res['Silent_Stare_min'].max() * 1.25, MIN_STARE_MIN + 10.0)
ax2.set_ylim(bottom=0.0, top=y2_max)
ax2.axhspan(MIN_STARE_MIN, y2_max, color='#6A3D9A', alpha=0.07, label='Compliant Silent Stare Region (≥30 min)')

# Secondary Top X-Axis: Altitude in Feet
alt_m_vals = df_res['Altitude_m'].values
alt_ft_vals = df_res['Altitude_ft'].values

def m_to_ft(x):
    return np.interp(x, alt_m_vals, alt_ft_vals)

def ft_to_m(x):
    return np.interp(x, alt_ft_vals, alt_m_vals)

ax_ft = ax1.secondary_xaxis('top', functions=(m_to_ft, ft_to_m))
ax_ft.set_xlabel('Altitude [Feet MSL]', fontweight='bold', fontsize=11, labelpad=8)
ax_ft.tick_params(axis='x')

# Consolidate Legends
handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(handles1 + handles2, labels1 + labels2, loc='upper center', bbox_to_anchor=(0.5, 0.95),
           framealpha=0.92, fontsize=8.5, ncol=2)

plt.title('Project Jatayu 2.0 - HE: Altitude Sensitivity Envelope ($h$)', fontweight='bold', fontsize=13, pad=38)

plt.figtext(
    0.5, 0.015,
    f"Held Constant: ICE = {P_ICE_BASE:.0f} kW  |  EM = {P_EM_BASE:.0f} kW  |  "
    f"Battery Pack = {E_BATT_BASE:.1f} kWh  |  Total Mass = {uav.MTOW:.0f} kg",
    ha='center', fontsize=9.5, style='italic',
    bbox=dict(boxstyle='square,pad=0.4', facecolor='#F0F0F0', edgecolor='#CCCCCC', lw=0.8)
)

fig.tight_layout(rect=[0.0, 0.05, 1.0, 0.95])
plt.savefig('plot_altitude_sensitivity.png', dpi=300)
plt.close(fig)
print("--> Sensitivity plot saved to plot_altitude_sensitivity.png")