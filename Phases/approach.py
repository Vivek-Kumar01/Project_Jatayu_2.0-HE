import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.descent as ds

m_initial = ds.m
m = ds.m
SoC = ds.SoC

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

t_step = 0.1 # sec
h_target = ms.obstacle_height # m

V = ds.V
h = ds.h

delta_Cd0_flaps = uav.Cd_0
k_uc = 3.16 * (10**-5)
delta_CD_0_gear = (m * 9.81 / uav.S)*k_uc*(uav.MTOW**-0.215)
Cd0_land = uav.Cd_0 + delta_CD_0_gear + delta_Cd0_flaps



safety_margin = 1.30 # ratio of velc to stall velc
V_stall_land = ((2 * m * 9.81)/(Atmosphere(h).density[0] * uav.Cl_max_Land * uav.S))**0.5
V_target = V_stall_land * safety_margin

t = 0
s_hor = 0

Phase_Success = True

t_baseline = ds.t_baseline + ds.t  # Continuous cumulative time elapsed before approach

while V > V_target:
    rho = Atmosphere(h).density[0]  # kg/m^3
    Cl = (m * 9.81) / (0.5 * rho * V**2 * uav.S)
    Cd = Cd0_land + uav.K * Cl**2
    D = 0.5 * rho * V**2 * uav.S * Cd

    a_net = -D/m # theta is zero, ROS = 0

    V += a_net * t_step
    t += t_step
    s_hor += V*t_step

    m_fuel = (P_ICE_Idle * (uav.BSFC_ICE_low / 3600)) * t_step / 1000  # kg
    m -= m_fuel

    # --- TELEMETRY LOGGING (Approach Deceleration) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = t_baseline + t
    ms.log_telemetry(cumulative_time, 'Approach_Decel', h, V, P_req, P_ICE_Idle, 0.0, fuel_remaining, SoC)

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break


theta = math.radians(3) # degrees
ROS = V * math.sin(theta) # m/s

eta_prop = 0.6

while h > h_target:
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
    s_hor += V * math.cos(theta) * t_step
    t += t_step

    # --- TELEMETRY LOGGING (Approach Glideslope) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    P_req = (D * V) / 1000.0  # kW
    P_ICE_current = (P_ICE_prop / 1000.0) + P_ICE_avionics
    cumulative_time = t_baseline + t
    ms.log_telemetry(cumulative_time, 'Approach_Glide', h, V, P_req, P_ICE_current, 0.0, fuel_remaining, SoC)

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break


if Phase_Success:
        print(f'''
    -------Approach Successful-------
    Horizontal Distance covered: {s_hor/1000:.2f} km, 
    Time: {t/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_initial - m:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {ds.SoC*100:.2f} %
    Final Velocity: {V:.2f} m/s,
    Percent ICE Power: {percent_power*100:.2f} %
''')


if not Phase_Success:
        print(f'''
    -------Approach Failed-------
    Horizontal Distance covered: {s_hor/1000:.2f} km, 
    Time: {t/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_initial - m:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg, 
    Final SoC: {ds.SoC*100:.2f} %
    Final Velocity: {V:.2f} m/s,
    Percent ICE Power: {percent_power*100:.2f} %
''')
