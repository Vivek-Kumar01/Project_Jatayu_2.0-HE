import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_out as co

# ------------------------------------------------------------------------------
# 1. LOITER ENVIRONMENTAL & AERODYNAMIC PARAMETERS
# ------------------------------------------------------------------------------
t_step = 1.0  # seconds
h = float(ms.loiter_altitude)
rho = float(Atmosphere(h).density[0])  # kg/m^3

safe_margin = 1.25  # Ratio of loiter velocity to stall velocity
cl_loiter = uav.Cl_max_clean / (safe_margin ** 2)
cd_loiter = uav.Cd_0 + uav.K * (cl_loiter ** 2)
n_prop = 0.72  # Propeller efficiency at low-speed loiter advance ratio

# Atmospheric derating for ICE at loiter altitude
P_ICE_avail = uav.get_ice_available_power(h)

# State Initializations from Cruise-Out
m_initial = float(co.m)  # kg
m = m_initial
SoC = float(co.SoC)
endurance = 0.0  # Total elapsed loiter time (s)

fraction_keep = 0.9  # Fuel burn margin for cruise-in
min_mass_cruisein = ms.m_zfw + ms.mass_reserve + (fraction_keep * co.m_fuel_burned)

Phase_Success = True
t_baseline = float(getattr(co, 'cumulative_time', co.t_baseline + co.time_cruise))

# Tactical Mission Boundaries
standoff_distance = 15000.0  # 15 km standoff gate (m)
P_avionics_kw = ms.power_avionics  # 0.350 kW

# Powertrain efficiency chains
eta_elec_traction = uav.eta_inverter * uav.eta_motor * uav.eta_discharge
eta_dcdc_hotel = uav.eta_DCDC_conv  # Isolated 700V -> 28V DC-DC converter efficiency
P_ICE_avionics = P_avionics_kw / uav.eta_alternator  # Mechanical shaft draw of engine alternator

# Guardrail check before silent ingress
if SoC < 0.85:
    print(f"WARNING: Battery SoC at standoff gate is {SoC*100:.1f}%. In-flight regeneration partial.")

# =============================================================================
# PART 1: SILENT ELECTRIC INGRESS (15 km to Target)
# =============================================================================
print("--> Initiating Phase 5A: Pure-Electric Silent Ingress (15 km)...")
dist_ingress = 0.0

while dist_ingress < standoff_distance and SoC > ms.SoC_min:
    vel_loiter = math.sqrt(2.0 * m * 9.81 / (rho * uav.S * cl_loiter))
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter
    P_prop_req = (D_loiter * vel_loiter) / 1000.0  # kW
    P_shaft_prop = P_prop_req / n_prop  # kW

    # ICE shut down & decoupled by sprag clutch
    P_ICE = 0.0
    P_electric_shaft = P_shaft_prop

    # Battery traction + avionics draw
    P_batt_traction = P_electric_shaft / eta_elec_traction
    P_batt_hotel = P_avionics_kw / eta_dcdc_hotel
    P_batt_total = P_batt_traction + P_batt_hotel  # Total kW from pack

    elec_energy_used = (P_batt_total * t_step) / 3600.0  # kWh
    SoC -= (elec_energy_used / ms.battery_capacity)

    dist_ingress += vel_loiter * t_step
    endurance += t_step

    # Telemetry logging (Pure electric operation)
    fuel_remaining = m - ms.m_zfw
    ms.log_telemetry(
        t_baseline + endurance, 'Silent_Ingress', h,
        vel_loiter, P_prop_req, 0.0, P_electric_shaft,
        fuel_remaining, SoC, 0.0, 0.0, 0.0
    )

print(f"    Ingress Complete: {dist_ingress/1000:.2f} km in {endurance/60:.1f} min | SoC: {SoC*100:.2f}%")

# Compute dynamic energy needed for Part 3 egress to preserve the 20% floor
t_est_egress = standoff_distance / vel_loiter
E_est_egress_kwh = (P_batt_total * t_est_egress) / 3600.0
SoC_egress_trigger = ms.SoC_min + (E_est_egress_kwh / ms.battery_capacity)

# =============================================================================
# PART 2: SILENT STARE OVER TARGET (Direct Reconnaissance)
# =============================================================================
print(f"--> Initiating Phase 5B: Direct Silent Stare on Station (Target Trigger SoC: {SoC_egress_trigger*100:.1f}%)...")
t_start_stare = endurance

while SoC > SoC_egress_trigger:
    vel_loiter = math.sqrt(2.0 * m * 9.81 / (rho * uav.S * cl_loiter))
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter
    P_prop_req = (D_loiter * vel_loiter) / 1000.0
    P_shaft_prop = P_prop_req / n_prop

    P_ICE = 0.0
    P_electric_shaft = P_shaft_prop
    P_batt_total = (P_electric_shaft / eta_elec_traction) + (P_avionics_kw / eta_dcdc_hotel)

    elec_energy_used = (P_batt_total * t_step) / 3600.0
    SoC -= (elec_energy_used / ms.battery_capacity)
    endurance += t_step

    fuel_remaining = m - ms.m_zfw
    ms.log_telemetry(
        t_baseline + endurance, 'Silent_Stare', h,
        vel_loiter, P_prop_req, 0.0, P_electric_shaft,
        fuel_remaining, SoC, 0.0, 0.0, 0.0
    )

print(f"    Silent Stare Complete: {(endurance - t_start_stare)/60:.1f} min | SoC: {SoC*100:.2f}%")

# =============================================================================
# PART 3: SILENT EGRESS (15 km Back to Standoff Gate)
# =============================================================================
print("--> Initiating Phase 5C: Silent Egress to Standoff Gate (15 km)...")
dist_egress = 0.0
t_start_egress = endurance

while dist_egress < standoff_distance and SoC > ms.SoC_min:
    vel_loiter = math.sqrt(2.0 * m * 9.81 / (rho * uav.S * cl_loiter))
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter
    P_prop_req = (D_loiter * vel_loiter) / 1000.0
    P_shaft_prop = P_prop_req / n_prop

    P_ICE = 0.0
    P_electric_shaft = P_shaft_prop
    P_batt_total = (P_electric_shaft / eta_elec_traction) + (P_avionics_kw / eta_dcdc_hotel)

    elec_energy_used = (P_batt_total * t_step) / 3600.0
    SoC -= (elec_energy_used / ms.battery_capacity)

    if SoC < ms.SoC_min:
        SoC = ms.SoC_min

    dist_egress += vel_loiter * t_step
    endurance += t_step

    fuel_remaining = m - ms.m_zfw
    ms.log_telemetry(
        t_baseline + endurance, 'Silent_Egress', h,
        vel_loiter, P_prop_req, 0.0, P_electric_shaft,
        fuel_remaining, SoC, 0.0, 0.0, 0.0
    )

t_end_electric = endurance
t_electric_total_min = t_end_electric / 60.0

print(f"    Egress Complete: {dist_egress/1000:.2f} km in {(endurance - t_start_egress)/60:.1f} min | Final Electric SoC: {SoC*100:.2f}%")

# =============================================================================
# PART 4: MID-AIR RELIGHT & SUSTAINED STANDOFF LOITER (Pure ICE)
# =============================================================================
print("--> Phase 5D: Mid-Air Relight Completed. Sprag clutch engaged.")
print("--> Commencing Sustained Standoff Loiter on Heavy Fuel...")

while m > min_mass_cruisein:
    vel_loiter = math.sqrt(2.0 * m * 9.81 / (rho * uav.S * cl_loiter))
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter
    P_prop_req = (D_loiter * vel_loiter) / 1000.0
    P_shaft_prop = P_prop_req / n_prop

    # Electric drive freewheeling, ICE supplies prop thrust + alternator
    P_electric_shaft = 0.0
    P_ICE_shaft_prop = P_shaft_prop / uav.eta_gearbox_ICE
    P_ICE_brake_total = P_ICE_shaft_prop + P_ICE_avionics

    if P_ICE_brake_total > P_ICE_avail:
        Phase_Success = False
        print("Error: Required ICE power exceeds available derated power during sustained loiter.")
        break

    # Load ratio and dynamic BSFC evaluation in loiter regime
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_loiter = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_loiter) / 3600.0     # g/s

    # Mass decrement per step
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    if m <= (ms.m_zfw + ms.mass_reserve):
        print("Emergency fuel reserve breached during loiter! RTB immediately.")
        break

    endurance += t_step

    fuel_remaining = m - ms.m_zfw
    ms.log_telemetry(
        t_baseline + endurance, 'Sustained_Loiter', h,
        vel_loiter, P_prop_req, P_ICE_shaft_prop, 0.0,
        fuel_remaining, SoC, xi_ICE, bsfc_loiter, m_dot_fuel_gps
    )

# ------------------------------------------------------------------------------
# 5. SUMMARY CALCULATIONS & EXPORT TO CRUISE-IN
# ------------------------------------------------------------------------------
t_ice_loiter_hr = (endurance - t_end_electric) / 3600.0
t_electric_loiter_hr = t_end_electric / 3600.0

cumulative_time = t_baseline + endurance
final_mass = m
final_velocity = vel_loiter
final_altitude = h
final_SoC = SoC

if Phase_Success:
    print(f'''
------------------ Complete Loiter Phase Summary ------------------
Status: SUCCESS
Total Loiter Endurance: {endurance / 3600.0:.2f} hours ({(endurance / 3600.0) * 60:.1f} mins)
  - Pure-Electric Silent Window: {t_electric_loiter_hr:.2f} hours ({t_electric_total_min:.1f} minutes)
  - Sustained ICE Standoff Loiter: {t_ice_loiter_hr:.2f} hours
Final Aircraft Mass: {m:.2f} kg
Final Pack SoC: {SoC * 100.0:.2f} % (Safety Floor: {ms.SoC_min * 100.0:.1f}%)
Usable Fuel Remaining for Cruise-In: {fuel_remaining - ms.mass_reserve:.2f} kg
Untouched Reserve Fuel: {ms.mass_reserve:.2f} kg
''')
else:
    print(f'''
------------------ Complete Loiter Phase Summary ------------------
Status: FAILED
Total Loiter Endurance: {endurance / 3600.0:.2f} hours
Final Mass: {m:.2f} kg | Final SoC: {SoC * 100.0:.2f} %\n''')