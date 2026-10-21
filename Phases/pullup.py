import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.takeoff as to

m_initial = to.m
SoC = to.SoC  # Initial state of charge for battery pack
m = to.m  # Initial mass of the UAV after takeoff

h = to.h

# Phase 1
rho = Atmosphere(h).density[0]  # kg/m^3
t_step = 0.1  # seconds

P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = P_ICE_full - P_ICE_avionics  # kW

P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

percent_ICE_power = 1 # Percentage of ICE power
percent_electric_power = 1 # Percentage of electric power

P_ICE = percent_ICE_power * P_ICE_full  # kW
P_electric = percent_electric_power * P_electric_full  # kW


DoH = P_electric / (P_ICE + P_electric)  # Hybridization factor (0 = pure ICE, 1 = pure electric)
P_shaft_prop = P_ICE * uav.eta_gearbox_ICE + P_electric  # kW

eta_prop = 0.68

P_available = P_shaft_prop * eta_prop  # kW

theta = 6.5 # degrees, climb angle
Cd_0_p1 = uav.Cd_0 + ms.delta_CD_0 + ms.delta_CD_0_flap

V = to.V  # m/s
s_roll = to.s_roll  # m
t = 0

Phase_Success = True

while h < ms.obstacle_height:
    L = m * 9.81 * math.cos(math.radians(theta))  # N
    Cl = L / (0.5 * rho * V**2 * uav.S)
    Cd = Cd_0_p1 + uav.K * Cl**2
    D = 0.5 * rho * V**2 * uav.S * Cd  # N
    T = P_available * 1000 / V  # N
    F_net = T - D - m * 9.81 * math.sin(math.radians(theta))  # N
    a_net = F_net / m  # m/s^2

    V = V + a_net * t_step  # m/s
    h = h + V * math.sin(math.radians(theta)) * t_step  # m
    t = t + t_step  # seconds
    s_roll = s_roll + V * math.cos(math.radians(theta)) * t_step  # m

    m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
    elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh

    SoC = SoC - (elec_energy_used / ms.battery_capacity)
    m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_max / 3600) * t_step / 1000)  # kg

    # --- TELEMETRY LOGGING (Pull-Up) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    P_req = (D * V) / 1000.0  # kW
    cumulative_time = (to.t + to.N) + t
    ms.log_telemetry(cumulative_time, 'Pull_Up', h, V, P_req, P_ICE, P_electric, fuel_remaining, SoC)

    if h < ms.obstacle_height and s_roll > ms.max_permissible_stot:
        print('''
        Error: Ground roll distance exceeded maximum permissible limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if SoC < ms.SoC_min:
        print('''
        Error: Battery state of charge dropped below minimum limit.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining for mission.
        MISSION FAILED.
        ''')
        Phase_Success = False
        break


if Phase_Success:
    print(f'''
    -------Pull Up Successful-------
    P_ICE: {P_ICE:.2f} kW, P_electric: {P_electric:.2f} kW, DoH: {DoH:.2f}
    Total phase 1 climb time: {t:.2f} seconds
    Total horizontal distance to clear obstacle: {s_roll:.2f} meters
    Final speed: {V:.2f} m/s
    Final altitude: {h:.2f} meters
    Fuel Burned: {(m_initial - m)*1000:.2f} g
    Remaining fuel mass: {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Remaining battery SoC: {SoC:.2%}
    ''')

if not Phase_Success:
    print(f'''
    -------Pull Up Failed-------
    P_ICE: {P_ICE:.2f} kW, P_electric: {P_electric:.2f} kW, DoH: {DoH:.2f}
    Total phase 1 climb time: {t:.2f} seconds
    Total horizontal distance to clear obstacle: {s_roll:.2f} meters
    Final speed: {V:.2f} m/s
    Final altitude: {h:.2f} meters
    Fuel Burned: {(m_initial - m)*1000:.2f} g
    Remaining fuel mass: {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Remaining battery SoC: {SoC:.2%}
    ''')


