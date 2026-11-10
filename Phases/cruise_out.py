import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_climb as cc

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM CRUISE-CLIMB END-OF-PHASE STATE
# ------------------------------------------------------------------------------
t_step = 1.0  # seconds
h = float(ms.cruise_altitude)
rho = float(Atmosphere(h).density[0])  # kg/m^3
standoff_dist = 15.0  # km buffer before target station

# Available ICE brake power at cruise altitude
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)     # kW

# Electric motor boost capacity
P_electric_full = (
    min(ms.power_battery_discharge, ms.power_inverter / uav.eta_inverter)
    * uav.eta_inverter
    * uav.eta_motor
)  # kW

n_prop = 0.76  # Propeller efficiency in steady cruise

dist_covered = 0.0  # km
m_inital = float(cc.m)
m = m_inital
SoC = float(cc.SoC)
time_cruise = 0.0  # seconds

# Optimal L/D clean aerodynamic parameters
cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * (cl_cruise ** 2)

t_baseline = float(getattr(cc, 'cumulative_time', cc.t_baseline + cc.t))
target_transit_dist = ms.cruise_distance - (cc.s_hor / 1000.0) - standoff_dist

Phase_Success = True

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CRUISE-OUT INTEGRATION
# ------------------------------------------------------------------------------
while dist_covered < target_transit_dist:
    P_elec_log = 0.0  # Negative = generator charging, Positive = battery assist

    # Instantaneous best-range airspeed and required thrust power
    vel_cruise = math.sqrt((2.0 * m * 9.81 / (rho * uav.S)) * math.sqrt(uav.K / uav.Cd_0))  # m/s
    D_cruise = 0.5 * rho * (vel_cruise ** 2) * uav.S * cd_cruise
    P_prop_req = (D_cruise * vel_cruise) / 1000.0  # Aerodynamic thrust power [kW]
    P_shaft_prop = P_prop_req / n_prop             # Required propeller shaft power [kW]

    # Total mechanical capability check
    if P_shaft_prop > (P_ICE_full * uav.eta_gearbox_ICE + P_electric_full):
        print('''
        Error: Required propeller shaft power exceeds maximum available hybrid power.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    # CASE A: ICE alone can sustain cruise (Optionally recharge battery)
    if P_shaft_prop <= (P_ICE_full * uav.eta_gearbox_ICE):
        DoH = 0.0

        if SoC < ms.SoC_max:
            # Generator load to replenish buffer pack
            P_gen_shaft_max = min(
                ms.power_battery_charge / (uav.eta_inverter * uav.eta_gearbox_gen),
                ms.power_inverter / uav.eta_gearbox_gen
            )
            P_ICE_prop_shaft = P_shaft_prop / uav.eta_gearbox_ICE
            P_ICE_demand = P_ICE_prop_shaft + P_gen_shaft_max

            if P_ICE_demand <= P_ICE_full:
                P_ICE = P_ICE_demand
                P_charging = P_gen_shaft_max * uav.eta_gearbox_gen * uav.eta_inverter
            else:
                P_ICE = P_ICE_full
                P_extra_shaft = P_ICE - P_ICE_prop_shaft
                P_charging = max(0.0, P_extra_shaft * uav.eta_gearbox_gen * uav.eta_inverter)

            energy_charged = (P_charging * uav.eta_charging * t_step) / 3600.0  # kWh
            SoC += (energy_charged / ms.battery_capacity)
            P_elec_log = -P_charging
        else:
            P_ICE = P_shaft_prop / uav.eta_gearbox_ICE
            P_elec_log = 0.0

    # CASE B: Prop shaft demand exceeds unassisted ICE rating (Electric Assist)
    else:
        P_ICE = P_ICE_full
        P_electric = P_shaft_prop - (P_ICE * uav.eta_gearbox_ICE)
        DoH = P_electric / (P_ICE + P_electric)

        elec_bus_demand = P_electric / (uav.eta_inverter * uav.eta_motor)
        if elec_bus_demand > ms.power_battery_discharge:
            print('''
            Error: Required electric assist exceeds continuous battery discharge rating.
            MISSION FAILED.
            ''')
            Phase_Success = False
            break

        if elec_bus_demand > ms.power_inverter:
            print('''
            Error: Required electric assist exceeds inverter rating.
            MISSION FAILED.
            ''')
            Phase_Success = False
            break

        elec_energy_used = (
            (P_electric / (uav.eta_inverter * uav.eta_motor * uav.eta_discharge)) * t_step
        ) / 3600.0  # kWh
        SoC -= (elec_energy_used / ms.battery_capacity)
        P_elec_log = P_electric

    # Engine load ratio and dynamic BSFC evaluation
    P_ICE_brake_total = P_ICE + P_ICE_avionics
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_cruise = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_cruise) / 3600.0     # g/s

    # Mass decrement per step
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    m -= m_fuel_step_kg

    dist_covered += (vel_cruise * t_step) / 1000.0  # km
    time_cruise += t_step                          # seconds

    # Telemetry logging
    fuel_remaining = m - ms.m_zfw
    cumulative_time = t_baseline + time_cruise
    ms.log_telemetry(
        cumulative_time, 'Cruise_Out', h, vel_cruise, P_prop_req, P_ICE, P_elec_log,
        fuel_remaining, SoC, xi_ICE, bsfc_cruise, m_dot_fuel_gps
    )

    if SoC < ms.SoC_min:
        print('''
        Error: Battery SoC dropped below minimum limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/loiter.py)
# ------------------------------------------------------------------------------
m_fuel_burned = m_inital - m
Phase_Success = Phase_Success and (dist_covered >= target_transit_dist)

cumulative_time = t_baseline + time_cruise
cumulative_distance = float(getattr(cc, 'cumulative_distance', cc.s_hor)) + (dist_covered * 1000.0)
final_mass = m
final_velocity = vel_cruise
final_altitude = h
final_SoC = SoC

if Phase_Success:
    print(f'''
    ------- Cruise Out Successful -------
    Target Transit Distance: {target_transit_dist:.2f} km | Covered: {dist_covered:.2f} km
    Transit Time: {time_cruise/60.0:.2f} min ({time_cruise/3600.0:.2f} hours)
    Cruise Velocity: {vel_cruise:.2f} m/s ({vel_cruise*3.6:.1f} km/h)
    Propeller Shaft Power: {P_shaft_prop:.2f} kW
    ICE Brake Load: {P_ICE_brake_total:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_cruise:.1f} g/kWh
    Final Aircraft Mass: {m:.2f} kg
    Fuel Burned: {m_fuel_burned:.2f} kg (Burn Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining:.2f} kg
    Buffer Battery SoC: {SoC*100.0:.2f} %\n''')
else:
    print(f'''
    ------- Cruise Out Failed -------
    Distance Covered: {dist_covered:.2f} km (Target: {target_transit_dist:.2f} km)
    Time: {time_cruise/60.0:.2f} min
    Final Mass: {m:.2f} kg | Final SoC: {SoC*100.0:.2f} %\n''')