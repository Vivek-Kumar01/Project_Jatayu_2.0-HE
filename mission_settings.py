import math
import jatayu_uav as uav

# UAV constraints and parameters
SoC_min = 0.2  # Minimum state of charge for battery pack
SoC_max = 0.9  # Maximum state of charge for battery pack
SoH = 1.0 # State of health for battery pack (1.0 = new, 0.8 = end of life)
reserve_fuel = 0.065 # Reserve fuel fraction for safety

power_avionics = 0.35 #kW

P_DCDC_rated = 0.50          # kW
battery_capacity = 40 #kWh
power_motor = 55 #kW
power_inverter = power_motor #kW
power_battery_discharge = uav.C_discharge_continuous_safe * battery_capacity #kW
power_battery_charge = uav.C_charge_continuous_safe * battery_capacity #kW

mass_motor = power_motor / uav.specific_power_motor #kg
mass_inverter = power_inverter / uav.specific_power_inverter #kg
mass_DCDC = P_DCDC_rated / uav.sp_power_DCDC
mass_electric_hardware = mass_motor + mass_inverter + mass_DCDC


mass_cells = battery_capacity * 1000 / uav.specific_energy_battery #kg
mass_battery_pack = uav.k_bat_pack * mass_cells #kg

mass_fuel_system = uav.M_power_pack - (mass_battery_pack + mass_electric_hardware) #kg
mass_fuel = mass_fuel_system / uav.k_fuel_sys #kg


# if mass_fuel > 0:
#     print(f'''
#     ================================================================
#                             MISSION JATAYU
#     ================================================================

#     UAV Charged and Fueled.
#     ''')
# else:
#     print("Error: Mass of fuel is negative. Please check the battery capacity and power requirements.")



# ==============================================================================
# PRE-FLIGHT CONFIGURATION & SIZING AUDIT
# ==============================================================================
if mass_fuel <= 0:
  print("\n" + "!" * 82)
  print(
      "FATAL SIZING ERROR: Mass of usable fuel is <= 0 kg. Check battery"
      " capacity!"
  )
  print("!" * 82 + "\n")
else:
  # Mass Breakdown & Aerodynamic Calculations
  mass_reserve = mass_fuel * reserve_fuel
  mass_mission_fuel = mass_fuel - mass_reserve
  m_zfw = uav.MTOW - mass_mission_fuel    # Zero Usable Fuel Weight
  wing_loading = uav.MTOW / uav.S  # kg/m²
  power_loading = uav.MTOW / (
      uav.P_ICE_rated + power_motor
  )  # kg/kW total installed
  ld_max_clean = 1.0 / (2.0 * math.sqrt(uav.Cd_0 * uav.K))

  print("=" * 82)
  print(f"{'PROJECT JATAYU: HYBRID UAV CONFIGURATION & SIZING AUDIT':^82}")
  print("=" * 82)

  # Category 1: Mass Fractions & Structural Limits
  print(f"{'--- MASS PROPERTIES & STRUCTURAL LIMITS ---':^82}")
  print(
      f"  MTOW:                {uav.MTOW:>8.2f} kg   │  Payload"
      f" Mass:        {uav.M_payload:>8.2f} kg"
  )
  print(
      f"  Structural Mass:     {uav.M_structure:>8.2f} kg   │  Avionics"
      f" Mass:       {uav.M_avionics:>8.2f} kg"
  )
  print(
      f"  Zero Fuel Wt (ZFW):  {m_zfw:>8.2f} kg   │  Max Landing Wt (MLW):"
      f" {uav.max_landing_mass:>8.2f} kg"
  )
  print("-" * 82)

  # Category 2: Aerodynamic Characteristics & Lift Limits
  print(f"{'--- AERODYNAMICS & LIFT COEFFICIENTS ---':^82}")
  print(
      f"  Wing Span (b):       {uav.b:>8.2f} m    │  Aspect Ratio"
      f" (AR):    {uav.AR:>8.2f}"
  )
  print(
      f"  Wing Area (S):       {uav.S:>8.2f} m²   │  Wing Loading (W/S):"
      f"   {wing_loading:>8.2f} kg/m²"
  )
  print(
      f"  Clean Zero-Lift Cd0: {uav.Cd_0:>8.4f}      │  Clean (L/D)max:"
      f"         {ld_max_clean:>8.2f}"
  )
  print(
      f"  CL_max (Clean):      {uav.Cl_max_clean:>8.2f}      │  CL_max"
      f" (Takeoff Flap):{uav.Cl_max_TO:>8.2f}"
  )
  print(
      f"  CL_ground (Roll):    {uav.Cl_ground:>8.2f}      │  CL_max (Landing"
      f" Flap):{uav.Cl_max_Land:>8.2f}"
  )
  print("-" * 82)

  # Category 3: ICE Sizing & Fuel Consumption Metrics
  print(f"{'--- COMPRESSION-IGNITION (CI) ENGINE ---':^82}")
  print(
      f"  ICE Installed Mass:  {uav.M_ICE_installed:>8.2f} kg   │  ICE Rated"
      f" Power:      {uav.P_ICE_rated:>8.2f} kW"
  )
  print(
      f"  ICE Rated Speed:     {uav.N_ICE_rated:>8.0f} RPM  │  Summing Gearbox"
      f" Mass: {uav.M_gearbox_hybrid:>8.2f} kg"
  )
  print(
      f"  BSFC (Opt Cruise):   {uav.BSFC_ICE_opt:>8.1f} g/kWh│  BSFC (100%"
      f" WOT Max):  {uav.BSFC_ICE_max:>8.1f} g/kWh"
  )
  print(
      f"  BSFC (Low Throttle): {uav.BSFC_ICE_low:>8.1f} g/kWh│  Total Usable"
      f" Fuel:   {mass_fuel:>8.2f} kg"
  )
  print(
      f"  Mission Fuel (93.5%):{mass_mission_fuel:>8.2f} kg   │  Reserve Fuel"
      f" (6.5%): {mass_reserve:>8.2f} kg"
  )
  print("-" * 82)

  # Category 4: Electric Propulsion & Energy Storage Metrics
  print(f"{'--- ELECTRICAL PROPULSION & BATTERY SYSTEM ---':^82}")
  print(
      f"  Motor Rated Power:   {power_motor:>8.2f} kW   │  Motor Specific"
      f" Power: {uav.specific_power_motor:>8.2f} kW/kg"
  )
  print(
      f"  Inverter Sizing:     {power_inverter:>8.2f} kW   │  Inverter Spec"
      f" Power:  {uav.specific_power_inverter:>8.2f} kW/kg"
  )
  print(
      f"  E-Drive Total Mass:  {mass_electric_hardware:>8.2f} kg   │  Battery"
      f" Pack Mass:   {mass_battery_pack:>8.2f} kg"
  )
  print(
      f"  Battery Capacity:    {battery_capacity:>8.2f} kWh  │  Cell Specific"
      f" Energy: {uav.specific_energy_battery:>8.2f} Wh/kg"
  )
  print(
      f"  Continuous Disch. C: {uav.C_discharge_continuous_safe:>8.2f} C    │ "
      f" Continuous Charge C: {uav.C_charge_continuous_safe:>8.2f} C"
  )
  print(
      f"  Max Cont. Disch. Pwr:{power_battery_discharge:>8.2f} kW   │  Max Cont."
      f" Charge Pwr: {power_battery_charge:>8.2f} kW"
  )
  print(
      f"  Total Installed W/P: {power_loading:>8.2f} kg/kW│  Avionics Base"
      f" Power:  {power_avionics:>8.2f} kW"
  )
  print(
        f"  Specific Power DC-DC: {uav.sp_power_DCDC:>8.2f} kW/kg│  DC-DC Rated"
        f" Power:  {P_DCDC_rated:>8.2f} kW"
    )
  print("=" * 82)
  print(f"{'PRE-FLIGHT AUDIT VERIFIED: AIRCRAFT GO FOR FLIGHT':^82}")
  print("=" * 82 + "\n")


# Take off constraints
altitude_takeoff = 0.0  # Takeoff altitude (m)
obstacle_height = altitude_takeoff + 15.0  # 50 ft screen height from sea level (m)
max_permissible_sroll = 600.0  # Max allowable ground roll (m)
max_permissible_stot = 1000.0  # Max total distance over 15 m obstacle (m)

rolling_friction_coefficient = 0.04  # Rolling friction coefficient for takeoff ground roll

k_uc = 3.16 * (10**-5)
delta_CD_0 = (uav.MTOW * 9.81/ uav.S)*k_uc*(uav.MTOW**-0.215)
delta_CD_0_flap = 0.0095
G = (16 * uav.h / uav.b)**2 / (1 + (16 * uav.h / uav.b)**2)
K_grd = (uav.k1 + G * uav.k3)
CD_takeoff = uav.Cd_0 + delta_CD_0 + delta_CD_0_flap + K_grd * (uav.Cl_ground**2)

# Cruise Out and In
cruise_altitude = 3000 #m
cruise_distance = 150 #km

# Loiter
loiter_altitude = cruise_altitude #m


# ==============================================================================
# MISSION TELEMETRY RECORDER (Expanded for Hybrid ICE BSFC Integration)
# ==============================================================================
telemetry = {
    'time_s': [],
    'phase': [],
    'altitude_m': [],
    'velocity_ms': [],
    'P_req_kW': [],
    'P_ICE_kW': [],
    'P_EM_kW': [],
    'fuel_mass_kg': [],
    'battery_soc': [],
    'xi_ICE': [],       # Engine throttle / load fraction (P_brake / P_avail)
    'BSFC_g_kWh': [],   # Instantaneous BSFC [g/kWh]
    'm_dot_fuel_gps': [] # Fuel burn rate [g/s]
}

def log_telemetry(t, phase, alt, v, p_req, p_ice, p_em, fuel, soc, xi_ice, bsfc, m_dot_fuel):
    """Appends one instantaneous simulation step to the telemetry log."""
    telemetry['time_s'].append(t)
    telemetry['phase'].append(phase)
    telemetry['altitude_m'].append(alt)
    telemetry['velocity_ms'].append(v)
    telemetry['P_req_kW'].append(p_req)
    telemetry['P_ICE_kW'].append(p_ice)
    telemetry['P_EM_kW'].append(p_em)
    telemetry['fuel_mass_kg'].append(fuel)
    telemetry['battery_soc'].append(soc)
    telemetry['xi_ICE'].append(xi_ice)
    telemetry['BSFC_g_kWh'].append(bsfc)
    telemetry['m_dot_fuel_gps'].append(m_dot_fuel)
