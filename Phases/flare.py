import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.approach as ap

m_initial = ap.m
m = ap.m
SoC = ap.SoC

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

t_step = 0.1 # sec

theta = ap.theta
n_flare = 1.05 # load factor at flare
V_flare = 1.23 * ap.V_stall_land # taking average velocity from 1.3 V_stall to 1.15 V_stall
R_f = (V_flare)**2 / ((n_flare - 1) * 9.81)  # in m,  Radius of curvature of path, laod factor = 1.2

h_flare = R_f * (1 - math.cos(theta)) # m

h = ap.h
V = ap.V

Cd0_land = ap.Cd0_land

ROS = V * math.sin(theta) # m/s

eta_prop = 0.6

s_hor_app = 0
t_app = 0
Phase_Success = True

t_baseline = ap.t_baseline + ap.t  # Continuous cumulative time elapsed before final approach

while h > ms.altitude_takeoff + h_flare:
    rho = Atmosphere(h).density[0]  # kg/m^3
    P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW

    Cl = (m * 9.81) / (0.5 * rho * V**2 * uav.S)
    Cd = Cd0_land + uav.K * Cl**2
    D = 0.5 * rho * V**2 * uav.S * Cd
    T_req = D - (m*9.81*math.sin(theta))

    P_avail = T_req * V
    P_prop_shaft = P_avail / eta_prop
    P_ICE_prop = P_prop_shaft / uav.eta_gearbox_ICE

    percent_power = (P_ICE_prop + P_ICE_avionics * 1000) / (P_ICE_full*1000)

    if percent_power > 1:
        print(f'''
        MISSION FAILED: Not enough power available for overcoming drag in landing.
        Percent Power: {percent_power:.2f}
''')
        Phase_Success = False
        break

    m_fuel = (((P_ICE_prop/1000) + P_ICE_avionics) * (uav.BSFC_ICE_low / 3600)) * t_step / 1000  # kg
    m -= m_fuel
    h -= ROS * t_step
    s_hor_app += V * math.cos(theta) * t_step
    t_app += t_step

    # --- TELEMETRY LOGGING (Final Approach) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    P_req = (D * V) / 1000.0  # kW
    P_ICE_current = (P_ICE_prop / 1000.0) + P_ICE_avionics
    cumulative_time = t_baseline + t_app
    ms.log_telemetry(cumulative_time, 'Final_Approach', h, V, P_req, P_ICE_current, 0.0, fuel_remaining, SoC)

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining in approach.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break


percent_idle_power = 6 # Percentage of total rated power at sea level
P_ICE_Idle = (percent_idle_power/100) * uav.P_ICE_rated # kW

P_ICE_shaft = P_ICE_Idle - P_ICE_avionics # kW
P_prop_shaft = P_ICE_shaft * uav.eta_gearbox_ICE

eta_prop = 0 # Assuming not thrust, no lift coeff on blades of prop, only drag
P_avail = P_prop_shaft * eta_prop

if P_prop_shaft < 0:
    print('''
WARNING: Idling power percentage is too low, unable to power avionics. Increase the percentage.
''')

s_hor_flare = R_f * math.sin(theta) # m

dist_flare = R_f * theta # m
t_flare = dist_flare / V_flare

m_fuel = (P_ICE_Idle * (uav.BSFC_ICE_low / 3600)) * t_flare / 1000  # kg
m -= m_fuel

V = 1.15 * ap.V_stall_land
h = ms.altitude_takeoff

# --- TELEMETRY LOGGING (Touchdown Instant at Runway Datum) ---
fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
D_flare = 0.5 * Atmosphere(h).density[0] * V**2 * uav.S * (Cd0_land + uav.K * ((m * 9.81)/(0.5 * Atmosphere(h).density[0] * V**2 * uav.S))**2)
P_req_flare = (D_flare * V) / 1000.0
cumulative_time = t_baseline + t_app + t_flare
ms.log_telemetry(cumulative_time, 'Flare_Touchdown', h, V, P_req_flare, P_ICE_Idle, 0.0, fuel_remaining, SoC)

if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
    print('''
    Error: Insufficient fuel remaining in flare.
    MISSION FAILED.
    ''')
    Phase_Success = False


if Phase_Success:
        print(f'''
    -------Final Approach & Flare Successful-------
    Approach Distance: {s_hor_app:.2f} m, 
    Approach Time: {t_app:.2f} sec,
    Flare Height from runway: {h_flare:.2f} m,
    Flare Distance: {s_hor_flare:.2f} m,
    Flare Time: {t_flare:.2f} sec, 
    Final Mass: {m:.2f} kg, 
    Total Fuel Burned (Final Approach + Flare): {(m_initial - m)*1000:.2f} g,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {SoC*100:.2f} %
    Final Velocity: {V:.2f} m/s,
''')


if not Phase_Success:
        print(f'''
    -------Final Approach & Flare Failed-------
    Approach Distance: {s_hor_app:.2f} m, 
    Approach Time: {t_app:.2f} sec,
    Flare Height from runway: {h_flare:.2f} m,
    Flare Distance: {s_hor_flare:.2f} m,
    Flare Time: {t_flare:.2f} sec, 
    Final Mass: {m:.2f} kg, 
    Total Fuel Burned (Final Approach + Flare): {(m_initial - m)*1000:.2f} g,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {SoC*100:.2f} %
    Final Velocity: {V:.2f} m/s,
''')
