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

# Fixed baseline energy storage (22 kWh pack)
ms.battery_capacity = 22.0  # kWh
ms.mass_cells = (ms.battery_capacity * 1000.0) / uav.specific_energy_battery
ms.mass_battery_pack = uav.k_bat_pack * ms.mass_cells

# Sweep DoH_P from 0.20 to 0.55
DOH_P_SWEEP = np.linspace(0.8, 0.9, 15)
sweep_records = []

print("=" * 96)
print(f"{'PROJECT JATAYU 2.0 - HE: POWER HYBRIDIZATION SWEEP (DoH_P vs. MISSION LOITER)':^96}")
print("=" * 96)

for idx, doh_p in enumerate(DOH_P_SWEEP):
    # --------------------------------------------------------------------------
    # 1. POWER SIZING & HARDWARE MUTATION
    # --------------------------------------------------------------------------
    p_em  = float(doh_p * P_TOTAL_TAKEOFF)
    p_ice = float(P_TOTAL_TAKEOFF - p_em)
    
    ms.power_motor = p_em
    ms.power_inverter = p_em
    ms.power_battery_discharge = uav.C_discharge_continuous_safe * ms.battery_capacity
    ms.power_battery_charge = uav.C_charge_continuous_safe * ms.battery_capacity
    uav.P_ICE_rated = p_ice
    
    # --------------------------------------------------------------------------
    # 2. COMPONENT MASS SCALING & AVAILABLE FUEL CALCULATION
    # --------------------------------------------------------------------------
    m_motor = p_em / uav.specific_power_motor
    m_inverter = p_em / uav.specific_power_inverter
    m_dcdc = ms.P_DCDC_rated / uav.sp_power_DCDC
    ms.mass_electric_hardware = m_motor + m_inverter + m_dcdc
    
    # Linear scale of ICE installed core + accessories
    m_ice_installed = M_ICE_INST_BASE * (p_ice / P_ICE_BASE)
    
    # Update dry structural weight and remaining fuel inside fixed MTOW budget
    fixed_dry_structural = (
        uav.M_structure + uav.M_propeller + uav.M_gearbox_hybrid +
        uav.M_torsional_damper + uav.M_clutch_decoupler + uav.M_payload +
        uav.M_avionics + m_ice_installed
    )
    m_power_pack_avail = uav.MTOW - fixed_dry_structural
    mass_fuel_sys = m_power_pack_avail - (ms.mass_battery_pack + ms.mass_electric_hardware)
    ms.mass_fuel = mass_fuel_sys / uav.k_fuel_sys
    
    if ms.mass_fuel <= 0.0:
        print(f"[{idx+1:02d}/{len(DOH_P_SWEEP):02d}] DoH_P: {doh_p:4.2f} --> SKIPPED (Mass budget violated)")
        continue
        
    ms.mass_reserve = ms.mass_fuel * ms.reserve_fuel
    ms.mass_mission_fuel = ms.mass_fuel - ms.mass_reserve
    ms.m_zfw = uav.MTOW - ms.mass_fuel

    # --------------------------------------------------------------------------
    # 3. THERMODYNAMIC BSFC PENALTY (Heywood Scaling Law)
    # --------------------------------------------------------------------------
    bsfc_scale = (P_ICE_BASE / p_ice) ** 0.12
    uav.BSFC_ICE_opt = BSFC_OPT_BASE * bsfc_scale
    uav.BSFC_ICE_max = BSFC_MAX_BASE * bsfc_scale
    uav.BSFC_ICE_low = BSFC_LOW_BASE * bsfc_scale

    # Dynamically scale the BSFC function polynomial in jatayu_uav
    def make_scaled_bsfc_func(scale_factor):
        def dynamic_bsfc(P_brake_kW, P_avail_kW):
            if P_avail_kW <= 0.0 or P_brake_kW <= 0.0:
                return uav.BSFC_ICE_low
            xi = float(np.clip(P_brake_kW / P_avail_kW, 0.05, 1.0))
            base_curve = (222.2 * (xi ** 2) - 311.1 * xi + 333.9)
            return float(np.clip(base_curve * scale_factor, uav.BSFC_ICE_opt, uav.BSFC_ICE_low))
        return dynamic_bsfc

    uav.get_ice_bsfc = make_scaled_bsfc_func(bsfc_scale)

    # --------------------------------------------------------------------------
    # 4. RESET TELEMETRY BUFFER (Full 12-Channel Schema)
    # --------------------------------------------------------------------------
    ms.telemetry = {
        'time_s': [], 'phase': [], 'altitude_m': [], 'velocity_ms': [],
        'P_req_kW': [], 'P_ICE_kW': [], 'P_EM_kW': [], 'fuel_mass_kg': [], 'battery_soc': [],
        'xi_ICE': [], 'BSFC_g_kWh': [], 'm_dot_fuel_gps': []
    }

    # --------------------------------------------------------------------------
    # 5. EXECUTE PHASES SILENTLY & VALIDATE Phase_Success
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
        print(f"[{idx+1:02d}/{len(DOH_P_SWEEP):02d}] DoH_P: {doh_p:4.2f} | ICE: {p_ice:5.1f} kW | EM: {p_em:4.1f} kW --> DISCARDED (Failed in {failed_phase_name})")
        continue

    # --------------------------------------------------------------------------
    # 6. EXTRACT MISSION METRICS
    # --------------------------------------------------------------------------
    df_sim = pd.DataFrame(ms.telemetry)
    total_hr = df_sim['time_s'].iloc[-1] / 3600.0
    
    df_stare = df_sim[df_sim['phase'] == 'Silent_Stare']
    t_silent_min = ((df_stare['time_s'].iloc[-1] - df_stare['time_s'].iloc[0]) / 60.0) if not df_stare.empty else 0.0

    df_loiter = df_sim[df_sim['phase'] == 'Sustained_Loiter']
    t_loiter_hr = ((df_loiter['time_s'].iloc[-1] - df_loiter['time_s'].iloc[0]) / 3600.0) if not df_loiter.empty else 0.0

    sweep_records.append({
        'DoH_P': doh_p,
        'P_ICE_kW': p_ice,
        'P_EM_kW': p_em,
        'Mass_Fuel_kg': ms.mass_fuel,
        'BSFC_opt': uav.BSFC_ICE_opt,
        'Total_Endurance_hr': total_hr,
        'Sustained_Loiter_hr': t_loiter_hr,
        'Silent_Stare_min': t_silent_min
    })

    print(f"[{idx+1:02d}/{len(DOH_P_SWEEP):02d}] DoH_P: {doh_p:4.2f} | ICE: {p_ice:5.1f} kW | EM: {p_em:4.1f} kW | Fuel: {ms.mass_fuel:5.1f} kg | BSFC_opt: {uav.BSFC_ICE_opt:5.1f} | Loiter: {t_loiter_hr:5.2f} hr | Tot: {total_hr:5.2f} hr")

# ------------------------------------------------------------------------------
# 7. EXPORT DATA & DUAL-AXIS TRADE-OFF PLOT (DoH_P)
# ------------------------------------------------------------------------------
if not sweep_records:
    print("\nError: All sweep design configurations failed or were skipped.")
    exit(1)

df_res = pd.DataFrame(sweep_records)
df_res.to_csv('hardware_sweep_doh_p_results.csv', index=False)
print("\n--> Results saved to hardware_sweep_doh_p_results.csv")

fig, ax1 = plt.subplots(figsize=(10, 6.2), dpi=300)

color_ice = '#1F78B4'    # Navy/Steel Blue for Sustained ICE Loiter
color_stare = '#33A02C'  # Forest Green for Silent Stare

# 1. Plot Left Axis: Sustained Loiter (Pure ICE)
line1 = ax1.plot(
    df_res['DoH_P'],
    df_res['Sustained_Loiter_hr'],
    marker='o',
    color=color_ice,
    lw=2.2,
    label='Sustained Standoff Loiter (Pure ICE) [Hours]'
)
ax1.set_xlabel('Power Degree of Hybridization ($DoH_P = P_{EM} / P_{total}$)', fontweight='bold', fontsize=11)
ax1.set_ylabel('Sustained Standoff Loiter [Hours]', color=color_ice, fontweight='bold', fontsize=11)
ax1.tick_params(axis='y', labelcolor=color_ice)
ax1.grid(True, linestyle=':', alpha=0.5)

# 2. Plot Right Axis: Silent Stare (Pure Electric)
ax2 = ax1.twinx()
line2 = ax2.plot(
    df_res['DoH_P'],
    df_res['Silent_Stare_min'],
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

y1_max = max(df_res['Sustained_Loiter_hr'].max() * 1.08, MIN_LOITER_HR + 4.0)
ax1.set_ylim(bottom=0.0, top=y1_max)
ax1.axhspan(MIN_LOITER_HR, y1_max, color='#33A02C', alpha=0.07, label='Compliant Standoff Loiter Region (≥12 hr)')

y2_max = max(df_res['Silent_Stare_min'].max() * 1.25, MIN_STARE_MIN + 10.0)
ax2.set_ylim(bottom=0.0, top=y2_max)
ax2.axhspan(MIN_STARE_MIN, y2_max, color='#6A3D9A', alpha=0.07, label='Compliant Silent Stare Region (≥30 min)')

# ------------------------------------------------------------------------------
# SECONDARY X-AXIS: Corresponding Motor Sizing (P_EM [kW])
# ------------------------------------------------------------------------------
doh_p_vals = df_res['DoH_P'].values
pem_vals = df_res['P_EM_kW'].values

def dohp_to_pem(x):
    return np.interp(x, doh_p_vals, pem_vals)

def pem_to_dohp(x):
    return np.interp(x, pem_vals, doh_p_vals)

ax_pem = ax1.secondary_xaxis('top', functions=(dohp_to_pem, pem_to_dohp))
ax_pem.set_xlabel('Installed Electric Motor Rating ($P_{EM}$) [kW]', fontweight='bold', fontsize=11, labelpad=8)
ax_pem.tick_params(axis='x')

# ------------------------------------------------------------------------------
# INVARIANT LABEL & COMBINED LEGEND
# ------------------------------------------------------------------------------
handles1, labels1 = ax1.get_legend_handles_labels()
handles2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(handles1 + handles2, labels1 + labels2, loc='upper center', bbox_to_anchor=(0.5, 0.95),
           framealpha=0.92, fontsize=8.5, ncol=2)

plt.title('Project Jatayu 2.0 - HE: Power Hybridization Envelope ($DoH_P$)', fontweight='bold', fontsize=13, pad=38)

plt.figtext(
    0.5, 0.015,
    f"Held Constant: Takeoff Power = {P_TOTAL_TAKEOFF:.1f} kW ($P_{{ICE}} + P_{{EM}}$)  |  "
    f"Battery Capacity = {ms.battery_capacity:.1f} kWh",
    ha='center', fontsize=9.5, style='italic',
    bbox=dict(boxstyle='square,pad=0.4', facecolor='#F0F0F0', edgecolor='#CCCCCC', lw=0.8)
)

fig.tight_layout(rect=[0.0, 0.05, 1.0, 0.95])
plt.savefig('plot_doh_p_tradeoff.png', dpi=300)
plt.close(fig)
print("--> Updated plot saved to plot_doh_p_tradeoff.png")