import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_out as co

# --- Loiter Environmental & Aerodynamic Constants ---
t_step = 1.0  # seconds
rho = Atmosphere(ms.loiter_altitude).density[0]  # kg/m^3

safe_margin = 1.25  # Ratio of loiter to stall velocity
cl_loiter = uav.Cl_max_clean / (safe_margin ** 2)
cd_loiter = uav.Cd_0 + uav.K * (cl_loiter ** 2)
n_prop = 0.72  # Propeller efficiency

# Atmospheric derating for ICE at 3000 m
rho_0 = Atmosphere(0).density[0]
P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / rho_0) - 0.132)  # Max available kW at 3000 m

# State Initializations from Cruise-Out
m_initial = co.m  # kg
m = m_initial
SoC = co.SoC
endurance = 0.0  # Total elapsed loiter time (s)

fraction_keep = 0.9  # Fuel burn margin for cruise-in
min_mass_cruisein = uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel) + fraction_keep * co.m_fuel_burned

Phase_Success = True
t_baseline = co.t_baseline + co.time_cruise

# --- Tactical Mission Boundaries ---
standoff_distance = 15000.0  # 15 km standoff gate (m)
P_avionics_kw = ms.power_avionics  # 0.350 kW

# Efficiency chains
eta_elec_traction = uav.eta_inverter * uav.eta_motor * uav.eta_discharge
eta_dcdc_hotel = uav.eta_DCDC_conv  # Isolated 700V -> 28V DC-DC converter efficiency
P_ICE_avionics = P_avionics_kw / uav.eta_alternator  # Mechanical shaft power needed by alternator

# Guardrail check before silent ingress
if SoC < 0.89:
    print(f"CRITICAL ERROR: Battery SoC at standoff gate is only {SoC*100:.1f}%. In-flight regeneration failed.")
    Phase_Success = False
    # handle abort

# =============================================================================
# PART 1: SILENT ELECTRIC INGRESS (15 km to Target)
# =============================================================================
print("--> Initiating Phase 5A: Pure-Electric Silent Ingress (15 km)...")
dist_ingress = 0.0

while dist_ingress < standoff_distance:
    vel_loiter = math.sqrt(2.0 * m * 9.81 / (rho * uav.S * cl_loiter))
    D_loiter = 0.5 * rho * (vel_loiter ** 2) * uav.S * cd_loiter
    P_prop_req = (D_loiter * vel_loiter) / 1000.0  # kW
    P_shaft_prop = P_prop_req / n_prop  # kW

    # ICE completely shut down & decoupled by sprag clutch
    P_ICE = 0.0
    P_electric_shaft = P_shaft_prop
    
    # Motor traction draw + avionics via 700V->28V DC-DC buck converter
    P_batt_traction = P_electric_shaft / eta_elec_traction
    P_batt_hotel = P_avionics_kw / eta_dcdc_hotel
    P_batt_total = P_batt_traction + P_batt_hotel  # Total kW from pack

    elec_energy_used = (P_batt_total * t_step) / 3600.0  # kWh
    SoC -= (elec_energy_used / ms.battery_capacity)

    dist_ingress += vel_loiter * t_step
    endurance += t_step

    # Telemetry
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel))
    ms.log_telemetry(t_baseline + endurance, 'Silent_Ingress', ms.loiter_altitude,
                     vel_loiter, P_prop_req, P_ICE, P_electric_shaft, fuel_remaining, SoC)

print(f"    Ingress Complete: {dist_ingress/1000:.2f} km in {endurance/60:.1f} min | SoC: {SoC*100:.2f}%")

# Compute dynamic energy needed for Part 3 egress to avoid breaching the 20% floor
t_est_egress = standoff_distance / vel_loiter
E_est_egress_kwh = (P_batt_total * t_est_egress) / 3600.0
SoC_egress_trigger = ms.SoC_min + (E_est_egress_kwh / ms.battery_capacity)

# =============================================================================
# PART 2: SILENT STARE OVER TARGET (Direct Reconnaissance)
# =============================================================================
print(f"--> Initiating Phase 5B: Direct Silent Stare on Station (Target SoC: {SoC_egress_trigger*100:.1f}%)...")
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

    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel))
    ms.log_telemetry(t_baseline + endurance, 'Silent_Stare', ms.loiter_altitude,
                     vel_loiter, P_prop_req, P_ICE, P_electric_shaft, fuel_remaining, SoC)

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
    
    # Enforce strict SoC floor clamp against numerical discretization
    if SoC < ms.SoC_min:
        SoC = ms.SoC_min

    dist_egress += vel_loiter * t_step
    endurance += t_step

    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel))
    ms.log_telemetry(t_baseline + endurance, 'Silent_Egress', ms.loiter_altitude,
                     vel_loiter, P_prop_req, P_ICE, P_electric_shaft, fuel_remaining, SoC)

# --- CAPTURE EXACT ELECTRIC PHASE END TIME ---
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

    # Electric system shut down; PMSM in zero-torque freewheel mode
    P_electric_shaft = 0.0
    # ICE delivers shaft power for prop + drives alternator for avionics
    P_ICE_shaft_prop = P_shaft_prop / uav.eta_gearbox_ICE
    P_ICE = P_ICE_shaft_prop + P_ICE_avionics

    if P_ICE > P_ICE_full:
        Phase_Success = False
        print("Error: Required ICE power exceeds available derated power during sustained loiter.")
        break

    # Fuel burn via low-BSFC map
    m_fuel_prop = (P_ICE_shaft_prop * (uav.BSFC_ICE_low / 3600.0)) * t_step / 1000.0
    m_fuel_avionics = (P_ICE_avionics * (uav.BSFC_ICE_low / 3600.0)) * t_step / 1000.0
    m -= (m_fuel_prop + m_fuel_avionics)

    if m <= (uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel)):
        print("Emergency fuel reserve breached during loiter! RTB immediately.")
        break

    endurance += t_step

    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1.0 - ms.reserve_fuel))
    ms.log_telemetry(t_baseline + endurance, 'Sustained_Loiter', ms.loiter_altitude,
                     vel_loiter, P_prop_req, P_ICE, P_electric_shaft, fuel_remaining, SoC)

# --- Summary Calculations ---
t_ice_loiter_hr = (endurance - t_end_electric) / 3600.0
t_electric_loiter_hr = t_end_electric / 3600.0

print(f'''
------------------ Complete Loiter Phase Summary ------------------
Status: {"SUCCESS" if Phase_Success else "FAILED"}
Total Loiter Endurance: {endurance / 3600.0:.2f} hours
  - Pure-Electric Silent Window: {t_electric_loiter_hr:.2f} hours ({t_electric_total_min:.1f} minutes)
  - Sustained ICE Standoff Loiter: {t_ice_loiter_hr:.2f} hours
Final Aircraft Mass: {m:.2f} kg
Final Pack SoC: {SoC * 100.0:.2f} % (Safety Floor: {ms.SoC_min * 100.0:.1f}%)
Usable Fuel Remaining for Cruise-In: {fuel_remaining:.2f} kg
''')