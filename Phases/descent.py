import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_in as ci

m_initial = ci.m
m = ci.m
SoC = ci.SoC

t_step = 0.1 # sec
h_target = ms.altitude_takeoff + 300 #m


percent_idle_power = 6 # Percentage of total rated power at sea level
P_ICE_Idle = (percent_idle_power/100) * uav.P_ICE_rated # kW

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

P_ICE_shaft = P_ICE_Idle - P_ICE_avionics # kW
P_prop_shaft = P_ICE_shaft * uav.eta_gearbox_ICE

eta_prop = 0 # Assuming not thrust, no lift coeff on blades of prop, only drag
P_avail = P_prop_shaft * eta_prop

if P_prop_shaft < 0:
    print('''
WARNING: Idling power percentage is too low, unable to power avionics. Increase the percentage.
''')

V = ci.vel_cruise
t = 0
s_hor = 0
h = ms.cruise_altitude

Phase_Success = True

t_baseline = ci.t_baseline + ci.time_cruise  # Continuous cumulative time elapsed before descent

while h > h_target:
    rho = Atmosphere(h).density[0]  # kg/m^3
    Cl = (m * 9.81) / (0.5 * rho * V**2 * uav.S)
    Cd = uav.Cd_0 + uav.K * Cl**2
    D = 0.5 * rho * V**2 * uav.S * Cd
    P_req = D * V

    ROS = P_req / (m * 9.81)
    theta = math.asin(ROS/V)

    h -= ROS * t_step
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    m_fuel = (P_ICE_Idle * (uav.BSFC_ICE_low / 3600)) * t_step / 1000  # kg
    m -= m_fuel

    # --- TELEMETRY LOGGING (Descent) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    cumulative_time = t_baseline + t
    ms.log_telemetry(cumulative_time, 'Descent', h, V, P_req / 1000.0, P_ICE_Idle, 0.0, fuel_remaining, SoC)

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

if Phase_Success:
        print(f'''
    -------Descent Successful-------
    Horizontal Distance covered: {s_hor/1000:.2f} km, 
    Time: {t/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_initial - m:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {SoC*100:.2f} %
    Final Velocity: {V:.2f} m/s,
''')


if not Phase_Success:
        print(f'''
    -------Descent Failed-------
    Horizontal Distance covered: {s_hor/1000:.2f} km, 
    Time: {t/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_initial - m:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {SoC*100:.2f} %
    Final Velocity: {V:.2f} m/s,
''')
