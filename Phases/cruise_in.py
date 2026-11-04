import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.loiter as lo

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM LOITER END-OF-PHASE STATE
# ------------------------------------------------------------------------------
descent_dist = 50.0  # km descent gate distance from base
t_step = 1.0         # seconds
h = float(ms.cruise_altitude)
rho = float(Atmosphere(h).density[0])  # kg/m^3

# Available ICE brake power at cruise altitude
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)     # kW

# Electric propulsion boost limits
P_electric_full = (
    min(ms.power_battery_discharge, ms.power_inverter / uav.eta_inverter)
    * uav.eta_inverter
    * uav.eta_motor
)  # kW

n_prop = 0.76  # Propeller efficiency

dist_covered = 0.0  # km
m_inital = float(lo.m)
m = m_inital
SoC = float(lo.SoC)
time_cruise = 0.0  # seconds

percent_electric_power = 0.10  # 10% electric power assist if SoC allows

cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * (cl_cruise ** 2)
Phase_Success = True

t_baseline = float(getattr(lo, 'cumulative_time', lo.t_baseline + lo.endurance))
target_transit_dist = ms.cruise_distance - descent_dist - (lo.standoff_distance / 1000.0)

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CRUISE-IN INTEGRATION
# ------------------------------------------------------------------------------
while dist_covered < target_transit_dist:
    vel_cruise = math.sqrt((2.0 * m * 9.81 / (rho * uav.S)) * math.sqrt(uav.K / uav.Cd_0))  # m/s
    D_cruise = 0.5 * rho * (vel_cruise ** 2) * uav.S * cd_cruise
    P_prop_req = (D_cruise * vel_cruise) / 1000.0  # kW
    P_shaft_prop = P_prop_req / n_prop             # Required propeller shaft power [kW]

    if P_shaft_prop > (P_ICE_full * uav.eta_gearbox_ICE + P_electric_full):
        print('''
        Error: Required propeller shaft power exceeds maximum available power.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    # Battery Assist Mode (if SoC is above floor)
    if SoC > ms.SoC_min:
        P_electric = percent_electric_power * P_electric_full
        P_ICE = (P_shaft_prop - P_electric) / uav.eta_gearbox_ICE

        if P_ICE > P_ICE_full:
            print('''
            Error: Required ICE power exceeds maximum available power.
            MISSION FAILED.
            ''')
            Phase_Success = False
            break

        if P_ICE < 0.0:
            print('''
            Bad Input: Required ICE power is negative. Too much electric power is being used.
            MISSION FAILED.
            ''')
            Phase_Success = False
            break

        elec_power_battery_bus = P_electric / (uav.eta_inverter * uav.eta_motor * uav.eta_discharge)
        elec_energy_used = (elec_power_battery_bus * t_step) / 3600.0  # kWh
        SoC -= (elec_energy_used / ms.battery_capacity)

    # Pure ICE Mode (SoC reached floor)
    else:
        P_ICE = P_shaft_prop / uav.eta_gearbox_ICE
        P_electric = 0.0

        if P_ICE > P_ICE_full:
            print('''
            Error: Required ICE power exceeds maximum available power and battery SoC is below minimum.
            MISSION FAILED.
            ''')
            Phase_Success = False
            break

    # Dynamic Engine Load & BSFC
    P_ICE_brake_total = P_ICE + P_ICE_avionics
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_cruise_in = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_cruise_in) / 3600.0     # g/s

    # Mass decrement
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    dist_covered += (vel_cruise * t_step) / 1000.0  # km
    time_cruise += t_step                          # seconds

    # Telemetry logging
    fuel_remaining = m - ms.m_zfw
    cumulative_time = t_baseline + time_cruise
    ms.log_telemetry(
        cumulative_time, 'Cruise_In', h, vel_cruise, P_prop_req, P_ICE, P_electric,
        fuel_remaining, SoC, xi_ICE, bsfc_cruise_in, m_dot_fuel_gps
    )

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/descent.py)
# ------------------------------------------------------------------------------
m_fuel_burned = m_inital - m
Phase_Success = Phase_Success and (dist_covered >= target_transit_dist)

cumulative_time = t_baseline + time_cruise
final_mass = m
final_velocity = vel_cruise
final_altitude = h
final_SoC = SoC

if Phase_Success:
    print(f'''
    ------- Cruise In Successful -------
    Target Transit Distance: {target_transit_dist:.2f} km | Covered: {dist_covered:.2f} km
    Transit Time: {time_cruise/60.0:.2f} min ({time_cruise/3600.0:.2f} hours)
    Cruise Velocity: {vel_cruise:.2f} m/s ({vel_cruise*3.6:.1f} km/h)
    Propeller Shaft Power: {P_shaft_prop:.2f} kW
    ICE Brake Load: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_cruise_in:.1f} g/kWh
    Final Aircraft Mass: {m:.2f} kg
    Fuel Burned: {m_fuel_burned:.2f} kg (Burn Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining - ms.mass_reserve:.2f} kg
    Untouched Safety Reserve: {ms.mass_reserve:.2f} kg
    Buffer Battery SoC: {SoC*100.0:.2f} %\n''')
else:
    print(f'''
    ------- Cruise In Failed -------
    Distance Covered: {dist_covered:.2f} km (Target: {target_transit_dist:.2f} km)
    Time: {time_cruise/60.0:.2f} min
    Final Mass: {m:.2f} kg | Final SoC: {SoC*100.0:.2f} %\n''')