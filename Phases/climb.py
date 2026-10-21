import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.pullup as pu

m_initial = pu.m
m = pu.m  # Initial mass of the UAV after pull-up
SoC = pu.SoC  # Initial state of charge for battery pack

t_step = 0.5  # seconds

h_target = h_target = ms.altitude_takeoff + (ms.cruise_altitude - ms.altitude_takeoff) / 2.0  # m

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

percent_ICE_power = 1 # Percentage of ICE power
percent_electric_power = 1 # Percentage of electric power

eta_prop = 0.74

V = pu.V
t = 0
Phase_Success = True

h = pu.h # m
s_hor = 0

t_baseline = pu.to.t + pu.to.N + pu.t  # Continuous time elapsed prior to climb

while h < h_target:
    rho = Atmosphere(h).density[0]  # kg/m^3

    P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW
    P_ICE_full -= P_ICE_avionics  # kW

    P_ICE = percent_ICE_power*P_ICE_full
    P_elec = percent_electric_power*P_electric_full

    DoH = P_elec/(P_elec + P_ICE)

    P_prop_shaft = P_ICE * uav.eta_gearbox_ICE + P_elec
    P_avail = P_prop_shaft * eta_prop
    
    Cl = (m * 9.81) / (0.5 * rho * V**2 * uav.S)
    Cd = uav.Cd_0 + uav.K * Cl**2
    D = (0.5 * rho * V**2 * uav.S)*Cd
    P_req = D*V

    R_C = (P_avail*1000 - P_req)/(m*9.81)
    theta = math.degrees(math.asin(R_C/V))

    h += R_C * t_step
    t += t_step
    s_hor += V * math.cos(math.radians(theta)) * t_step

    m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
    elec_energy_used = ((P_elec/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh

    SoC = SoC - (elec_energy_used / ms.battery_capacity)
    m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_max / 3600) * t_step / 1000)  # kg

    # --- TELEMETRY LOGGING (Climb) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    cumulative_time = t_baseline + t
    ms.log_telemetry(cumulative_time, 'Climb', h, V, P_req / 1000.0, P_ICE, P_elec, fuel_remaining, SoC)

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
    -------Climb Successful-------
    EOP P_ICE: {P_ICE:.2f} kW, P_electric: {P_elec:.2f} kW
    Total climb time: {t/60:.2f} mins
    Horizontal Distance Travelled: {s_hor/1000:.2f} km
    Final speed: {V:.2f} m/s
    Fuel burned: {m_initial - m:.2f} kg
    Remaining fuel mass: {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Remaining battery SoC: {SoC:.2%}
    ''')

if not Phase_Success:
    print(f'''
    -------Climb Failed-------
    EOP P_ICE: {P_ICE:.2f} kW, P_electric: {P_elec:.2f} kW
    Total climb time: {t/60:.2f} mins
    Horizontal Distance Travelled: {s_hor/1000:.2f} km
    Final speed: {V:.2f} m/s
    Fuel burned: {m_initial - m:.2f} kg
    Remaining fuel mass: {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Remaining battery SoC: {SoC:.2%}
    ''')
