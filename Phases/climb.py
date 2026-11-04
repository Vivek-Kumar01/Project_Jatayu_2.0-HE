import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.pullup as pu

# ------------------------------------------------------------------------------
# 1. INITIALIZE FROM PULL-UP END-OF-PHASE STATE
# ------------------------------------------------------------------------------
m_initial = float(pu.m)
m = float(pu.m)
SoC = float(pu.SoC)
h = float(pu.h)
V = float(getattr(pu, 'final_velocity', pu.V))
s_hor = 0.0

t_step = 0.5  # seconds
eta_prop = 0.74
Phase_Success = True
t = 0.0

# Midway climb target (1,500 m MSL for 3,000 m cruise)
h_target = ms.altitude_takeoff + (ms.cruise_altitude - ms.altitude_takeoff) / 2.0

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

# Continuous battery assist limits during climb
P_electric_full = (
    min(ms.power_battery_discharge, ms.power_inverter / uav.eta_inverter)
    * uav.eta_inverter
    * uav.eta_motor
)  # kW

percent_ICE_power = 1.0       # Full throttle climb
percent_electric_power = 1.0  # Continuous electric climb assist

# Cumulative baseline time elapsed before starting climb
t_baseline = float(getattr(pu, 'cumulative_time', pu.to.t + pu.to.N + pu.t))

# ------------------------------------------------------------------------------
# 2. TIME-MARCHING CLIMB INTEGRATION (Screen Height to 1,500 m)
# ------------------------------------------------------------------------------
while h < h_target:
    rho = float(Atmosphere(h).density[0])  # kg/m^3

    # Dynamic altitude derating of internal combustion engine
    P_ICE_avail = uav.get_ice_available_power(h)
    P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)

    P_ICE = percent_ICE_power * P_ICE_full
    P_elec = percent_electric_power * P_electric_full

    DoH = P_elec / (P_elec + P_ICE) if (P_elec + P_ICE) > 0.0 else 0.0

    # Total shaft and thrust power delivered to propeller
    P_prop_shaft = (P_ICE * uav.eta_gearbox_ICE) + P_elec
    P_avail = P_prop_shaft * eta_prop  # kW

    # Aerodynamic drag and required power at current weight and speed
    Cl = (m * 9.81) / (0.5 * rho * (V ** 2) * uav.S)
    Cd = uav.Cd_0 + uav.K * (Cl ** 2)
    D = 0.5 * rho * (V ** 2) * uav.S * Cd  # N
    P_req = D * V                          # W

    # Rate of climb and flight path angle
    excess_power_W = (P_avail * 1000.0) - P_req
    R_C = excess_power_W / (m * 9.81)      # m/s

    if R_C <= 0.0:
        print(f"Climb failed: Rate of climb collapsed to {R_C:.2f} m/s at {h:.1f} m")
        Phase_Success = False
        break

    sin_theta = max(-1.0, min(1.0, R_C / V))
    theta = math.degrees(math.asin(sin_theta))

    # Engine load ratio and dynamic BSFC evaluation
    P_ICE_brake_total = P_ICE + P_ICE_avionics
    xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
    bsfc_climb = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
    m_dot_fuel_gps = (P_ICE_brake_total * bsfc_climb) / 3600.0     # g/s

    # Mass and battery energy decrement per time step
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0
    elec_power_battery_bus = P_elec / (uav.eta_inverter * uav.eta_motor * uav.eta_discharge)
    elec_energy_step_kWh = (elec_power_battery_bus * t_step) / 3600.0

    # State advancement
    h += R_C * t_step
    t += t_step
    s_hor += V * math.cos(math.radians(theta)) * t_step

    m -= m_fuel_step_kg
    SoC -= (elec_energy_step_kWh / ms.battery_capacity)

    # Telemetry logging
    fuel_remaining = m - ms.m_zfw
    cumulative_time = t_baseline + t
    ms.log_telemetry(
        cumulative_time, 'Climb', h, V, P_req / 1000.0, P_ICE, P_elec,
        fuel_remaining, SoC, xi_ICE, bsfc_climb, m_dot_fuel_gps
    )

    if SoC < ms.SoC_min:
        print('''
        Error: Battery state of charge dropped below minimum limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if m < ms.m_zfw:
        print('''
        Error: Insufficient fuel remaining for mission.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

# ------------------------------------------------------------------------------
# 3. EXPORT VARIABLES FOR DOWNSTREAM PHASES (Phases/cruise_climb.py)
# ------------------------------------------------------------------------------
cumulative_time = t_baseline + t
cumulative_distance = float(getattr(pu, 'cumulative_distance', pu.s_roll)) + s_hor
final_mass = m
final_velocity = V
final_altitude = h
final_SoC = SoC

if Phase_Success:
    print(f'''
    ------- Climb Successful -------
    EOP P_ICE: {P_ICE:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_climb:.1f} g/kWh
    EOP P_electric: {P_elec:.2f} kW | Shaft Power: {P_prop_shaft:.2f} kW | DoH: {DoH*100:.1f} %
    Rate of Climb (EOP): {R_C:.2f} m/s
    Total Climb Time: {t/60.0:.2f} mins
    Horizontal Distance Travelled: {s_hor/1000.0:.2f} km
    Final Altitude: {h:.2f} m (Target: {h_target:.1f} m)
    Final Velocity: {V:.2f} m/s ({V*3.6:.1f} km/h)
    Fuel Burned: {(m_initial - m):.2f} kg
    Usable Fuel Remaining: {fuel_remaining - ms.mass_reserve:.2f} kg
    Remaining Battery SoC: {SoC*100.0:.2f} %\n''')
else:
    print(f'''
    ------- Climb Failed -------
    EOP P_ICE: {P_ICE:.2f} kW | P_electric: {P_elec:.2f} kW
    Total Climb Time: {t/60.0:.2f} mins
    Horizontal Distance: {s_hor/1000.0:.2f} km
    Final Altitude: {h:.2f} m
    Battery SoC: {SoC*100.0:.2f} %\n''')