# This contains the parameters for the UAV.
import math
import numpy as np
from ambiance import Atmosphere

# Fixed masses
MTOW = 1000 #kg
M_structure = 275 #kg
M_propeller = 14.5 #kg
M_gearbox_hybrid = 16.0  # kg (Combined PRGB, EM mating gear, housing, bearings, and oil)
M_torsional_damper = 3.5  # kg (CI crankshaft isolator / dual-mass flywheel)
M_clutch_decoupler = 2.5  # kg (Freewheel / sprag clutch unit)
M_payload = 200 #kg
M_ICE_dry = 95.0  # kg (Bare engine core)
M_ICE_accessories = 21.5  # kg (Cooling, oil, intake/exhaust, ECU/FADEC)
M_ICE_installed = M_ICE_dry + M_ICE_accessories
M_avionics = 45 #kg

M_power_pack = MTOW - (M_structure + M_propeller + M_gearbox_hybrid + M_torsional_damper + M_clutch_decoupler + M_payload + M_ICE_installed + M_avionics)  # kg

# Fixed body-wing parameters
b = 18 #m
h = 1.6 #m
S = 22 #m^2
lamb = 0.45 # Taper ratio
k1 = 0.009 # proportianlity constant between increase in parasitic drag and CL^2
AR = b**2/S
delta = 0.02 # induced drag factor
e = 1/(1+delta) # span efficiency factor
k3 = 1 / (math.pi * AR * e) # induced drag coeff prop constant
K = k1 + k3 # induced drag factor in drag polar equation
Cd_0 = 0.0265 # zero-lift parasitic drag coefficient
Cl_ground = 0.35
Cl_max_ground_TO = 1.25
Cl_max_TO = 1.7
Cl_max_Land = 1.85
Cl_max_clean = 1.4
D_prop = 1.75 #m
N_prop = 2400 #RPM (constant speed propeller)

# Fixed engine parameters
P_ICE_rated = 60.0  # kW (Continuous rated brake shaft power)
N_ICE_rated = 3800.0  # RPM (Engine crankshaft speed at rated power)
# Torque_ICE_rated = (
#     P_ICE_rated * 1000.0
# ) / (  # ~150.8 N·m
#     2 * 3.14159 * (N_ICE_rated / 60.0)
# )
BSFC_ICE_low = 340  # g/kWh (BSFC at low power)
BSFC_ICE_opt = 225.0  # g/kWh (Optimal cruise BSFC on Jet-A/diesel)
BSFC_ICE_max = 245.0  # g/kWh (BSFC at 100% rated power at sea level)
# V_disp = 1.8  # Litres (Swept displacement ~1.6 - 2.0 L for 60 kW NA CI)

specific_power_motor = (
    4.0  # kW/kg (Continuous rated, high-performance PMSM motor)
)
specific_power_inverter = 12.0  # kW/kg (High-voltage SiC inverter)
specific_energy_battery = (
    200.0  # Wh/kg (Pack-level Li-ion / NMC cells with BMS & casing)
)
sp_power_DCDC = 0.50         # kW/kg (Mil-COTS isolated high-voltage aerospace standard)

# Overhead factors
k_bat_pack = 1.18  # 18% overhead for module casing, BMS, busbars, cooling
k_fuel_sys = 1.05  # 5% overhead for wing bladders, pumps, baffles, unusable trapped fuel

# General-purpose safe design constants for a 200 Wh/kg NMC pack:
C_discharge_continuous_safe = 2.0  # 1/h (Continuous max discharge)
C_charge_continuous_safe = 1.0  # 1/h (Continuous max charge)

# Efficiency parameters
eta_gearbox_ICE = 0.97  # Mechanical efficiency: Engine to Prop

eta_gearbox_gen = 0.95  # Mechanical efficiency: Engine -> Prop Shaft -> Generator
eta_motor = 0.95  # PMSM electric motor efficiency
eta_inverter = 0.97  # SiC inverter efficiency

eta_discharge = 0.9 # battery discharge efficiency
eta_charging = 0.95 # battery charging efficiency

eta_alternator = 0.8  # Alternator efficiency (engine to electrical)
eta_DCDC_conv = 0.95 # DCDC converter efficiency

# Landing characteristics
max_landing_mass_percent = 100  # %
max_landing_mass = (max_landing_mass_percent/100) * MTOW


# Baseline reference constants
rho_0 = 1.2250  # kg/m^3 (ISA Sea Level)

def get_ice_available_power(h_m, P_rated=P_ICE_rated):
    """
    Computes naturally aspirated CI aero-engine brake power (kW) at altitude h_m.
    Applies the classic Gagg-Farrar density derating formulation.
    """
    atmos = Atmosphere(h_m)
    rho = float(atmos.density[0]) if hasattr(atmos.density, '__len__') else float(atmos.density)
    sigma = rho / rho_0
    factor = 1.132 * sigma - 0.132
    return max(0.0, P_rated * factor)

def get_ice_bsfc(P_brake_kW, P_avail_kW):
    """
    Brake Specific Fuel Consumption (BSFC) in g/kWh for a 4-stroke CI aero-engine
    governed at constant 3800 crankshaft RPM (constant-speed propeller).
    Anchors: minimum 225 g/kWh at xi = 0.70; 245 g/kWh at xi = 1.00.
    """
    if P_avail_kW <= 0.0 or P_brake_kW <= 0.0:
        return 340.0  # Motoring / unpowered idle limit

    xi = float(np.clip(P_brake_kW / P_avail_kW, 0.05, 1.0))
    bsfc = 222.2 * (xi ** 2) - 311.1 * xi + 333.9
    return float(np.clip(bsfc, BSFC_ICE_opt, BSFC_ICE_low))