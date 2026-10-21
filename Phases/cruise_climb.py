import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.climb as cl

m_initial = cl.m
m = cl.m
SoC = cl.SoC

t_step = 0.1

h_target = ms.cruise_altitude
guess_mass_fuel = 10 #kg to set target cruise velocity
vel_cruise = (2 * (m-guess_mass_fuel) * 9.81 / (Atmosphere(ms.cruise_altitude).density[0] * uav.S) * (uav.K / uav.Cd_0)**0.5)**0.5

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

percent_ICE_power = 1 # Percentage of ICE power
percent_electric_power = 0 # Percentage of electric power

eta_prop = 0.74

dV_dh = (vel_cruise - cl.V)/(ms.cruise_altitude - cl.h_target)


V = cl.V
h = cl.h
t = 0
s_hor = cl.s_hor
Phase_Success = True

t_baseline = cl.t_baseline + cl.t  # Continuous cumulative time elapsed before cruise-climb

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

    R_C = (P_avail*1000 - P_req)/(m*(9.81 + V * dV_dh))
    theta = math.asin(R_C/V)

    V += R_C*(dV_dh) * t_step
    h += R_C * t_step
    s_hor += V*math.cos(theta)*t_step
    t += t_step
    

    m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
    elec_energy_used = ((P_elec/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh

    SoC = SoC - (elec_energy_used / ms.battery_capacity)
    m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_max / 3600) * t_step / 1000)  # kg

    # --- TELEMETRY LOGGING (Cruise Climb) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    cumulative_time = t_baseline + t
    ms.log_telemetry(cumulative_time, 'Cruise_Climb', h, V, P_req / 1000.0, P_ICE, P_elec, fuel_remaining, SoC)

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
    -------Cruise Climb Successful-------
    EOP P_ICE: {P_ICE:.2f} kW, P_electric: {P_elec:.2f} kW
    Total climb time: {t/60:.2f} mins
    Final speed: {V:.2f} m/s
    Horizontal distance travelled: {s_hor/1000:.2f} km
    Fuel burned: {m_initial - m:.2f} kg
    Remaining fuel mass: {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Remaining battery SoC: {SoC:.2%}
    ''')

if not Phase_Success:
    print(f'''
    -------Cruise Climb Failed-------
    EOP P_ICE: {P_ICE:.2f} kW, P_electric: {P_elec:.2f} kW
    Total climb time: {t/60:.2f} mins
    Final speed: {V:.2f} m/s
    Horizontal distance travelled: {s_hor/1000:.2f} km
    Fuel burned: {m_initial - m:.2f} kg
    Remaining fuel mass: {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg
    Remaining battery SoC: {SoC:.2%}
    ''')
