import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms
import Phases.cruise_climb as cc

# Cruise Out simulation setting
t_step = 1.0  # seconds
rho = Atmosphere(ms.cruise_altitude).density[0]  # kg/m^3
standoff_dist = 15 # km

P_ICE_full = uav.P_ICE_rated * ((1.132 * rho / Atmosphere(0).density[0]) - 0.132)  # kW

P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW
P_ICE_full = P_ICE_full - P_ICE_avionics  # kW

P_electric_full = (min(ms.power_battery_discharge, ms.power_inverter/uav.eta_inverter))*uav.eta_inverter*uav.eta_motor  # kW

n_prop = 0.76  # Propeller efficiency

dist_covered = 0.0  # km
m_inital = cc.m # kg
m = m_inital  # kg
SoC = cc.SoC
time_cruise = 0.0  # seconds

cl_cruise = math.sqrt(uav.Cd_0 / uav.K)
cd_cruise = uav.Cd_0 + uav.K * cl_cruise**2

t_baseline = cc.t_baseline + cc.t  # Continuous cumulative time elapsed before cruise out

while dist_covered < ms.cruise_distance - (cc.s_hor)/1000 - standoff_dist:
    P_elec_log = 0.0  # Track electrical power for logging (-ve for charging, +ve for discharging)

    vel_cruise = (2 * m * 9.81 / (rho * uav.S) * (uav.K / uav.Cd_0)**0.5)**0.5  # m/s
    D_cruise = 0.5 * rho * vel_cruise**2 * uav.S * cd_cruise
    P_prop_req = D_cruise * vel_cruise / 1000  # kW
    P_shaft_prop = P_prop_req / n_prop  # kW

    if P_shaft_prop > P_ICE_full * uav.eta_gearbox_ICE + P_electric_full:
        print('''
        Error: Required propeller shaft power exceeds maximum available power.
        MISSION FAILED.
        ''')
        break

    if P_shaft_prop <= P_ICE_full * uav.eta_gearbox_ICE:
        DoH = 0.0  # Pure ICE operation

        if SoC < ms.SoC_max:
            P_ICE = P_shaft_prop / uav.eta_gearbox_ICE + min(ms.power_battery_charge/(uav.eta_inverter*uav.eta_gearbox_gen), ms.power_inverter/uav.eta_gearbox_gen)  # kW

            if P_ICE < P_ICE_full:
                P_charging = min(ms.power_battery_charge/(uav.eta_inverter*uav.eta_gearbox_gen), ms.power_inverter/uav.eta_gearbox_gen) * uav.eta_gearbox_gen * uav.eta_inverter  # kW
            else:
                P_ICE = P_ICE_full  # kW
                P_extra = P_ICE - P_shaft_prop / uav.eta_gearbox_ICE  # kW
                P_charging = P_extra*uav.eta_gearbox_gen*uav.eta_inverter  # kW

            energy_charged = (P_charging * uav.eta_charging * t_step) / 3600  # kWh
            SoC = SoC + (energy_charged / ms.battery_capacity)

            m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
            m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_max / 3600) * t_step / 1000)

            P_elec_log = -P_charging  # Negative denotes generator feeding power into pack
        else:
            P_ICE = P_shaft_prop / uav.eta_gearbox_ICE  # kW
            m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
            m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_max / 3600) * t_step / 1000)

            P_elec_log = 0.0

    elif P_shaft_prop > P_ICE_full * uav.eta_gearbox_ICE:
        P_ICE = P_ICE_full  # kW
        P_electric = (P_shaft_prop - P_ICE * uav.eta_gearbox_ICE)  # kW
        DoH = P_electric / (P_ICE + P_electric)  # Hybridization factor (0 = pure ICE, 1 = pure electric)

        if P_electric/(uav.eta_inverter * uav.eta_motor) > ms.power_battery_discharge:
            print('''
            Error: Required electric power exceeds battery discharge limit.
            MISSION FAILED.
            ''')
            break

        if P_electric/(uav.eta_inverter * uav.eta_motor) > ms.power_inverter:
            print('''
            Error: Required electric power exceeds inverter limit.
            MISSION FAILED.
            ''')
            break

        elec_energy_used = ((P_electric/(uav.eta_inverter*uav.eta_motor*uav.eta_discharge)) * t_step) / 3600  # kWh
        SoC = SoC - (elec_energy_used / ms.battery_capacity)

        m_fuel = (P_ICE * (uav.BSFC_ICE_max / 3600)) * t_step / 1000  # kg
        m = m - m_fuel - ((P_ICE_avionics * uav.BSFC_ICE_max / 3600) * t_step / 1000)

        P_elec_log = P_electric  # Positive denotes battery discharge assist

    if SoC < ms.SoC_min:
        print('''
        Error: Battery SoC dropped below minimum.
        MISSION FAILED.
        ''')
        break

    if m < uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel):
        print('''
        Error: Insufficient fuel remaining.
        MISSION FAILED.
        ''')
        break

    dist_covered = dist_covered + (vel_cruise * t_step) / 1000  # km
    time_cruise = time_cruise + t_step  # seconds

    # --- TELEMETRY LOGGING (Cruise Out) ---
    fuel_remaining = m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel))
    cumulative_time = t_baseline + time_cruise
    ms.log_telemetry(cumulative_time, 'Cruise_Out', ms.cruise_altitude, vel_cruise, P_prop_req, P_ICE, P_elec_log, fuel_remaining, SoC)
    
print("\n")

m_fuel_burned = m_inital - m
Phase_Success = dist_covered >= ms.cruise_distance - (cc.s_hor)/1000  - standoff_dist

if Phase_Success:
    print(f'''
    -------Cruise Out Successful-------
    Distance covered: {dist_covered:.2f} km, 
    Time: {time_cruise/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_fuel_burned:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg,
    Final SoC: {SoC*100:.2f} %
    Final Velocity: {vel_cruise:.2f} m/s,
''')

else:

    print(f'''
    -------Cruise Out Failed-------
    Distance covered: {dist_covered:.2f} km, 
    Time: {time_cruise/60:.2f} min, 
    Final Mass: {m:.2f} kg, 
    Fuel Burned: {m_fuel_burned:.2f} kg,
    Fuel left (excluding reserve): {m - (uav.MTOW - ms.mass_fuel * (1 - ms.reserve_fuel)):.2f} kg,
    Final SoC: {SoC*100:.2f} %
    Final Velocity: {vel_cruise:.2f} m/s,
''')


