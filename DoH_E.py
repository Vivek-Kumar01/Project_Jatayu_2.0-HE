from contextlib import redirect_stdout
import importlib
import math
import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Suppress the massive pre-flight sizing audit printed on import
with open(os.devnull, 'w', encoding='utf-8') as fnull:
    with redirect_stdout(fnull):
        import jatayu_uav as uav
        import mission_settings as ms

# List of phase modules in exact chronological sequence
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
    'Phases.touchdown',
]

# ------------------------------------------------------------------------------
# BASELINE CALIBRATION ANCHORS
# ------------------------------------------------------------------------------
P_ICE_BASE      = float(uav.P_ICE_rated)
P_EM_BASE       = float(ms.power_motor)
P_TOTAL_TAKEOFF = P_ICE_BASE + P_EM_BASE

M_ICE_INST_BASE = float(uav.M_ICE_dry + uav.M_ICE_accessories)
BSFC_OPT_BASE   = float(uav.BSFC_ICE_opt)
BSFC_MAX_BASE   = float(uav.BSFC_ICE_max)
BSFC_LOW_BASE   = float(uav.BSFC_ICE_low)

# ------------------------------------------------------------------------------
# 1. DEFINE BATTERY CAPACITY SWEEP RANGE (kWh)
# ------------------------------------------------------------------------------
E_BATT_SWEEP = np.linspace(20.0, 35.0, 16)  # Expanded range to inspect full trade envelope
sweep_records = []

print('=' * 88)
print(f"{'PROJECT JATAYU 2.0 - HE: HARDWARE SWEEP (DoH_E vs. MISSION METRICS)':^88}")
print('=' * 88)

for idx, e_batt in enumerate(E_BATT_SWEEP):
    # --------------------------------------------------------------------------
    # A. MUTATE SIZING PARAMETERS IN MISSION_SETTINGS
    # --------------------------------------------------------------------------
    ms.battery_capacity = float(e_batt)

    # Re-evaluate dependent continuous power bounds
    ms.power_battery_discharge = uav.C_discharge_continuous_safe * ms.battery_capacity
    ms.power_battery_charge = uav.C_charge_continuous_safe * ms.battery_capacity

    # Re-evaluate dependent masses
    ms.mass_cells = (ms.battery_capacity * 1000.0) / uav.specific_energy_battery  # kg
    ms.mass_battery_pack = uav.k_bat_pack * ms.mass_cells                         # kg
    ms.mass_fuel_system = uav.M_power_pack - (ms.mass_battery_pack + ms.mass_electric_hardware)
    ms.mass_fuel = ms.mass_fuel_system / uav.k_fuel_sys                          # kg

    # Feasibility check: fuel depletion under fixed MTOW power pack budget
    if ms.mass_fuel <= 0.0:
        print(f"[{idx+1:02d}/{len(E_BATT_SWEEP):02d}] E_batt = {e_batt:4.1f} kWh --> SKIPPED (Fuel mass <= 0 kg)")
        continue

    ms.mass_reserve = ms.mass_fuel * ms.reserve_fuel
    ms.mass_mission_fuel = ms.mass_fuel - ms.mass_reserve
    ms.m_zfw = uav.MTOW - ms.mass_fuel

    # Calculate Energy Degree of Hybridization (DoH_E)
    E_fuel_kWh = ms.mass_fuel * 11.97  # LHV Jet-A1 ~ 11.97 kWh/kg
    doh_e = ms.battery_capacity / (ms.battery_capacity + E_fuel_kWh)

    # --------------------------------------------------------------------------
    # B. RESET TELEMETRY BUFFER (Full 12-Channel Schema)
    # --------------------------------------------------------------------------
    ms.telemetry = {
        'time_s': [],
        'phase': [],
        'altitude_m': [],
        'velocity_ms': [],
        'P_req_kW': [],
        'P_ICE_kW': [],
        'P_EM_kW': [],
        'fuel_mass_kg': [],
        'battery_soc': [],
        'xi_ICE': [],
        'BSFC_g_kWh': [],
        'm_dot_fuel_gps': []
    }

    # --------------------------------------------------------------------------
    # C. EXECUTE PHASES SILENTLY & VALIDATE Phase_Success
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
        print(
            f"[{idx+1:02d}/{len(E_BATT_SWEEP):02d}] Batt: {e_batt:4.1f} kWh | Fuel: {ms.mass_fuel:5.1f} kg "
            f"--> DISCARDED (Failed in {failed_phase_name})"
        )
        continue

    # --------------------------------------------------------------------------
    # D. EXTRACT METRICS FROM TELEMETRY
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)

    total_flight_time_hr = df_sim['time_s'].iloc[-1] / 3600.0

    df_stare = df_sim[df_sim['phase'] == 'Silent_Stare']
    t_silent_min = (
        (df_stare['time_s'].iloc[-1] - df_stare['time_s'].iloc[0]) / 60.0
        if not df_stare.empty
        else 0.0
    )

    df_loiter = df_sim[df_sim['phase'] == 'Sustained_Loiter']
    t_loiter_hr = (
        (df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0
        if not df_loiter.empty
        else 0.0
    )

    fuel_burned_kg = df_sim['fuel_mass_kg'].iloc[0] - df_sim['fuel_mass_kg'].iloc[-1]

    # Dynamic BSFC evaluation during powered ICE phases
    df_ice = df_sim[df_sim['P_ICE_kW'] > 0.5]
    avg_bsfc = df_ice['BSFC_g_kWh'].mean() if not df_ice.empty else 0.0

    sweep_records.append({
        'E_batt_kWh': e_batt,
        'Mass_Batt_kg': ms.mass_battery_pack,
        'Mass_Fuel_kg': ms.mass_fuel,
        'DoH_E_percent': doh_e * 100.0,
        'Total_Endurance_hr': total_flight_time_hr,
        'Silent_Stare_min': t_silent_min,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Fuel_Burned_kg': fuel_burned_kg,
        'Mean_BSFC_g_kWh': avg_bsfc
    })

    print(
        f"[{idx+1:02d}/{len(E_BATT_SWEEP):02d}] Batt: {e_batt:4.1f} kWh | Fuel: {ms.mass_fuel:5.1f} kg | "
        f"DoH_E: {doh_e*100:4.1f}% | Silent: {t_silent_min:5.1f} min | Loiter: {t_loiter_hr:5.2f} hr | "
        f"BSFC: {avg_bsfc:5.1f} g/kWh | Total: {total_flight_time_hr:5.2f} hr"
    )

# ------------------------------------------------------------------------------
# 2. EXPORT & DUAL-AXIS TRADE-OFF PLOT (DoH_E)
# ------------------------------------------------------------------------------
if not sweep_records:
    print("\nError: All sweep design configurations failed or were skipped.")
    exit(1)

df_results = pd.DataFrame(sweep_records)
df_results.to_csv('hardware_sweep_doh_results.csv', index=False)
print('\n--> Sweep data exported to hardware_sweep_doh_results.csv')

fig, ax1 = plt.subplots(figsize=(10, 6.2), dpi=300)

color_ice = '#1F78B4'    # Navy/Steel Blue for Sustained ICE Loiter
color_stare = '#33A02C'  # Forest Green for Silent Stare

# 1. Left Axis: Sustained Loiter (ICE)
line1 = ax1.plot(
    df_results['E_batt_kWh'],
    df_results['Sustained_Loiter_hr'],
    marker='o',
    color=color_ice,
    lw=2.2,
    label='Sustained Standoff Loiter (Pure ICE) [Hours]'
)
ax1.set_xlabel('Installed Battery Pack Capacity [kWh]', fontweight='bold', fontsize=11)
ax1.set_ylabel('Sustained Standoff Loiter [Hours]', color=color_ice, fontweight='bold', fontsize=11)
ax1.tick_params(axis='y', labelcolor=color_ice)
ax1.grid(True, linestyle=':', alpha=0.5)

# 2. Right Axis: Silent Stare (EM)
ax2 = ax1.twinx()
line2 = ax2.plot(
    df_results['E_batt_kWh'],
    df_results['Silent_Stare_min'],
    marker='s',
    color=color_stare,
    lw=2.2,
    linestyle='--',
    label='Silent Stare on Station (Pure Electric) [Minutes]'
)
ax2.set_ylabel('Silent Stare Duration [Minutes]', color=color_stare, fontweight='bold', fontsize=11)
ax2.tick_params(axis='y', labelcolor=color_stare)

# ------------------------------------------------------------------------------
# CONSTRAINT THRESHOLDS & FEASIBLE REGION SHADING
# ------------------------------------------------------------------------------
MIN_LOITER_HR = 12.0
MIN_STARE_MIN = 30.0

line_t1 = ax1.axhline(MIN_LOITER_HR, color='#E31A1C', linestyle=':', lw=1.6, label=f'Req: Min Loiter ({MIN_LOITER_HR:.0f} hr)')
line_t2 = ax2.axhline(MIN_STARE_MIN, color='#6A3D9A', linestyle=':', lw=1.6, label=f'Req: Min Stare ({MIN_STARE_MIN:.0f} min)')

y1_max = max(df_results['Sustained_Loiter_hr'].max() * 1.08, MIN_LOITER_HR + 3.0)
ax1.set_ylim(bottom=0.0, top=y1_max)
ax1.axhspan(MIN_LOITER_HR, y1_max, color='#33A02C', alpha=0.07, label='Compliant Standoff Loiter Region (≥12 hr)')

y2_max = max(df_results['Silent_Stare_min'].max() * 1.15, MIN_STARE_MIN + 10.0)
ax2.set_ylim(bottom=0.0, top=y2_max)
ax2.axhspan(MIN_STARE_MIN, y2_max, color='#6A3D9A', alpha=0.07, label='Compliant Silent Stare Region (≥30 min)')

# ------------------------------------------------------------------------------
# SECONDARY X-AXIS: Energy Degree of Hybridization (DoH_E %)
# ------------------------------------------------------------------------------
doh_vals = df_results['DoH_E_percent'].values
ebatt_vals = df_results['E_batt_kWh'].values

def ebatt_to_dohe(x):
    return np.interp(x, ebatt_vals, doh_vals)

def dohe_to_ebatt(x):
    return np.interp(x, doh_vals, ebatt_vals)

ax_doh = ax1.secondary_xaxis('top', functions=(ebatt_to_dohe, dohe_to_ebatt))
ax_doh.set_xlabel('Energy Degree of Hybridization ($DoH_E$) [%]', fontweight='bold', fontsize=11, labelpad=8)
ax_doh.tick_params(axis='x')

# ------------------------------------------------------------------------------
# INVARIANT LABEL & COMBINED LEGEND
# ------------------------------------------------------------------------------
handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(handles1 + handles2, labels1 + labels2, loc='upper center', bbox_to_anchor=(0.5, 0.95),
           framealpha=0.92, fontsize=8.5, ncol=2)

plt.title('Project Jatayu 2.0 - HE: Energy Hybridization Envelope ($DoH_E$)', fontweight='bold', fontsize=13, pad=38)

plt.figtext(
    0.5, 0.015,
    f"Held Constant: Power-Pack Mass Budget = {uav.M_power_pack:.1f} kg  |  "
    f"Takeoff Power = {P_TOTAL_TAKEOFF:.1f} kW ({P_ICE_BASE:.0f} kW ICE + {ms.power_motor:.0f} kW EM)",
    ha='center', fontsize=9.5, style='italic',
    bbox=dict(boxstyle='square,pad=0.4', facecolor='#F0F0F0', edgecolor='#CCCCCC', lw=0.8)
)

fig.tight_layout(rect=[0.0, 0.05, 1.0, 0.95])
plt.savefig('plot_doh_tradeoff_pareto.png', dpi=300)
plt.close(fig)
print('--> Updated trade-off Pareto curve saved as plot_doh_tradeoff_pareto.png')