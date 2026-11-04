import math
from ambiance import Atmosphere
import jatayu_uav as uav
import mission_settings as ms

# ------------------------------------------------------------------------------
# 1. INITIAL RUNWAY & ENVIRONMENTAL CONDITIONS
# ------------------------------------------------------------------------------
h = ms.altitude_takeoff
rho = float(Atmosphere(h).density[0])  # kg/m^3
t_step = 0.1  # seconds

# Available engine brake power at runway altitude
P_ICE_avail = uav.get_ice_available_power(h)

# Continuous electrical parasitics drawn via engine-driven alternator
P_ICE_avionics = ms.power_avionics / uav.eta_alternator  # kW

# Net mechanical power available for propeller shaft after alternator parasitic
P_ICE_full = max(0.0, P_ICE_avail - P_ICE_avionics)  # kW

# Electric propulsion sizing limit (battery discharge vs inverter throughput)
P_electric_full = (
    min(ms.power_battery_discharge, ms.power_inverter / uav.eta_inverter)
    * uav.eta_inverter
    * uav.eta_motor
)  # kW

# Throttle settings for maximum takeoff performance
percent_ICE_power = 1.0       # Full throttle ICE
percent_electric_power = 1.0  # Full boost electric assist

P_ICE = percent_ICE_power * P_ICE_full          # Propulsion brake shaft power [kW]
P_electric = percent_electric_power * P_electric_full  # Motor shaft power [kW]

# Drivetrain power combination through reduction/summing gearbox
P_shaft_prop = (P_ICE * uav.eta_gearbox_ICE) + P_electric  # kW
DoH = P_electric / (P_ICE + P_electric) if (P_ICE + P_electric) > 0.0 else 0.0

# ------------------------------------------------------------------------------
# 2. INSTANTANEOUS ENGINE LOAD & FUEL CONSUMPTION METRICS
# ------------------------------------------------------------------------------
P_ICE_brake_total = P_ICE + P_ICE_avionics  # Total crankshaft brake demand [kW]
xi_ICE = min(1.0, P_ICE_brake_total / P_ICE_avail) if P_ICE_avail > 0.0 else 0.0
bsfc_takeoff = uav.get_ice_bsfc(P_ICE_brake_total, P_ICE_avail)  # g/kWh
m_dot_fuel_gps = (P_ICE_brake_total * bsfc_takeoff) / 3600.0     # g/s

# ------------------------------------------------------------------------------
# 3. AERODYNAMIC & GROUND ROLL THRESHOLDS
# ------------------------------------------------------------------------------
Merit_Factor = 0.7  # Static propeller figure of merit
Static_thrust = (Merit_Factor ** (2 / 3)) * (
    2 * rho * (math.pi / 4 * (uav.D_prop ** 2)) * ((P_shaft_prop * 1000.0) ** 2)
) ** (1 / 3)

N = 1.0  # Rotation transition duration [seconds]
V_stall = math.sqrt((2.0 * uav.MTOW * 9.81) / (rho * uav.S * uav.Cl_max_ground_TO))  # m/s
V_LO = 1.15 * V_stall  # Liftoff speed (1.15 * V_stall) [m/s]

prop_eff_liftoff = 0.65
Thrust_liftoff = (prop_eff_liftoff * P_shaft_prop * 1000.0) / V_LO  # N

def thrust_available(V):
    """Quadratic decay of static thrust towards high-speed advance."""
    return Static_thrust + (Thrust_liftoff - Static_thrust) * ((V / V_LO) ** 2)

# ------------------------------------------------------------------------------
# 4. TIME-MARCHING GROUND ROLL INTEGRATION
# ------------------------------------------------------------------------------
t = 0.0        # seconds
s_roll = 0.0   # m
V = 0.0        # m/s
m = float(uav.MTOW)  # kg
SoC = ms.SoC_max     # Battery state of charge (fraction 0-1)

while V < V_LO:
    T = thrust_available(V)
    D = 0.5 * rho * (V ** 2) * uav.S * ms.CD_takeoff
    W = m * 9.81
    L = 0.5 * rho * (V ** 2) * uav.S * uav.Cl_ground
    
    F_net = T - D - (W - L) * ms.rolling_friction_coefficient
    a_net = F_net / m

    # Fuel mass consumed in step (propulsion + avionics alternator)
    m_fuel_step_kg = (m_dot_fuel_gps * t_step) / 1000.0  # kg

    # Electrical energy extracted from battery (including inverter, motor, discharge loss)
    elec_power_battery_bus = P_electric / (uav.eta_inverter * uav.eta_motor * uav.eta_discharge)  # kW
    elec_energy_step_kWh = (elec_power_battery_bus * t_step) / 3600.0  # kWh

    # Kinematics and mass updates
    s_roll += V * t_step
    V += a_net * t_step
    t += t_step
    m -= m_fuel_step_kg
    SoC -= (elec_energy_step_kWh / ms.battery_capacity)

    # Telemetry logging
    fuel_remaining = m - ms.m_zfw
    P_req = (D * V) / 1000.0  # kW
    ms.log_telemetry(
        t, 'Takeoff', h, V, P_req, P_ICE, P_electric,
        fuel_remaining, SoC, xi_ICE, bsfc_takeoff, m_dot_fuel_gps
    )

    if s_roll > ms.max_permissible_sroll:
        print("Takeoff failed: maximum permissible ground roll exceeded.")
        break

    if SoC < ms.SoC_min:
        print("Takeoff failed: battery SoC dropped below minimum.")
        break

    if m < ms.m_zfw:
        print("Takeoff failed: insufficient fuel remaining.")
        break

# ------------------------------------------------------------------------------
# 5. ROTATION SEGMENT DYNAMICS (N seconds)
# ------------------------------------------------------------------------------
s_roll += N * V_LO
t_rotation_start = t
t += N

# Energy & fuel consumed during rotation
m_fuel_rotation_kg = (m_dot_fuel_gps * N) / 1000.0
elec_energy_rot_kWh = (elec_power_battery_bus * N) / 3600.0

m -= m_fuel_rotation_kg
SoC -= (elec_energy_rot_kWh / ms.battery_capacity)

fuel_remaining = m - ms.m_zfw
P_req_rotation = (0.5 * rho * (V_LO ** 2) * uav.S * ms.CD_takeoff * V_LO) / 1000.0  # kW

ms.log_telemetry(
    t, 'Takeoff_Rotation', h, V_LO, P_req_rotation, P_ICE, P_electric,
    fuel_remaining, SoC, xi_ICE, bsfc_takeoff, m_dot_fuel_gps
)

# ------------------------------------------------------------------------------
# 6. EXPORT VARIABLES FOR DOWNSTREAM PHASES
# ------------------------------------------------------------------------------
Phase_Success = (s_roll <= ms.max_permissible_sroll) and (SoC >= ms.SoC_min)

cumulative_time = t
cumulative_distance = s_roll
final_mass = m
final_velocity = V_LO
final_altitude = h
final_SoC = SoC

if not Phase_Success:
    print(f"Takeoff failed: ground roll {s_roll:.1f} m exceeds limit {ms.max_permissible_sroll:.1f} m")
else:
    print(f'''
    ------- Takeoff Successful -------
    P_ICE: {P_ICE:.2f} kW | Load Ratio (xi): {xi_ICE:.2f} | BSFC: {bsfc_takeoff:.1f} g/kWh
    P_electric: {P_electric:.2f} kW | Shaft Power: {P_shaft_prop:.2f} kW | DoH: {DoH*100:.1f} %
    Total Ground Roll Distance: {s_roll:.2f} m (Limit: {ms.max_permissible_sroll:.0f} m)
    Liftoff Velocity (V_LO): {V_LO:.2f} m/s ({V_LO*3.6:.1f} km/h)
    Time to Liftoff: {t:.2f} seconds
    Fuel Burned: {(uav.MTOW - m)*1000.0:.2f} g (Rate: {m_dot_fuel_gps:.2f} g/s)
    Usable Fuel Remaining: {fuel_remaining - ms.mass_reserve:.2f} kg
    Battery SoC After Takeoff: {SoC*100.0:.2f} %\n''')