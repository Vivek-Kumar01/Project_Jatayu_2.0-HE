# This contains the parameters for the UAV.
import math
import numpy as np
from ambiance import Atmosphere

# ==============================================================================
# CONFIGURATION SWITCH & SCALING CONTROLS
# ==============================================================================
edit_baseline = True  # Set to True to resize the ICE from the baseline 60 kW
P_ICE_target = 50.0   # Target rated brake power in kW (e.g., 45.0, 50.0, etc.)

# Baseline 60 kW Reference Engine Anchors
P_ICE_rated_base = 60.0          # kW
M_ICE_dry_base = 95.0            # kg
M_ICE_accessories_base = 21.5    # kg
BSFC_ICE_opt_base = 225.0        # g/kWh (Minimum at xi = 0.70)
BSFC_ICE_max_base = 245.0        # g/kWh (At 100% rated at sea level)
BSFC_ICE_low_base = 340.0        # g/kWh (Motoring/idle threshold)

if edit_baseline:
    P_ICE_rated = float(P_ICE_target)
    scale_factor = P_ICE_rated / P_ICE_rated_base
    
    # 1. Mass Scaling
    # Dry core scales close to linearly (~0.95 exponent); accessories scale with heat rejection (~0.80)
    M_ICE_dry = M_ICE_dry_base * (scale_factor ** 0.95)
    M_ICE_accessories = M_ICE_accessories_base * (scale_factor ** 0.80)
    
    # 2. Heywood Thermodynamic BSFC Scaling (exponent -0.12)
    # Downsizing worsens BSFC due to higher relative surface-to-volume heat loss and friction
    bsfc_mult = (P_ICE_rated_base / P_ICE_rated) ** 0.12
    BSFC_ICE_opt = BSFC_ICE_opt_base * bsfc_mult
    BSFC_ICE_max = BSFC_ICE_max_base * bsfc_mult
    BSFC_ICE_low = BSFC_ICE_low_base * bsfc_mult
else:
    P_ICE_rated = P_ICE_rated_base
    M_ICE_dry = M_ICE_dry_base
    M_ICE_accessories = M_ICE_accessories_base
    BSFC_ICE_opt = BSFC_ICE_opt_base
    BSFC_ICE_max = BSFC_ICE_max_base
    BSFC_ICE_low = BSFC_ICE_low_base

M_ICE_installed = M_ICE_dry + M_ICE_accessories

# ==============================================================================
# FIXED MASSES & TOTAL AVAILABLE POWER PACK
# ==============================================================================
MTOW = 1000.0              # kg
M_structure = 275.0        # kg
M_propeller = 14.5         # kg
M_gearbox_hybrid = 16.0    # kg (Combined PRGB, EM mating gear, housing, bearings, and oil)
M_torsional_damper = 3.5   # kg (CI crankshaft isolator / dual-mass flywheel)
M_clutch_decoupler = 2.5   # kg (Freewheel / sprag clutch unit)
M_payload = 200.0          # kg
M_avionics = 45.0          # kg

# Power pack budget remaining for: Battery Pack + Motor/Inverter + DC-DC + Fuel System + Fuel
M_power_pack = MTOW - (
    M_structure 
    + M_propeller 
    + M_gearbox_hybrid 
    + M_torsional_damper 
    + M_clutch_decoupler 
    + M_payload 
    + M_ICE_installed 
    + M_avionics
)  # kg

# ==============================================================================
# FIXED BODY-WING AERODYNAMICS
# ==============================================================================
b = 18.0                   # m
h = 1.6                    # m
S = 22.0                   # m^2
lamb = 0.45                # Taper ratio
k1 = 0.009                 # Proportionality constant between increase in parasitic drag and CL^2
AR = b**2 / S
delta = 0.02               # Induced drag factor
e = 1.0 / (1.0 + delta)    # Span efficiency factor
k3 = 1.0 / (math.pi * AR * e)
K = k1 + k3                # Induced drag factor in drag polar
Cd_0 = 0.022              # Zero-lift parasitic drag coefficient
Cl_ground = 0.35
Cl_max_ground_TO = 1.25
Cl_max_TO = 1.70
Cl_max_Land = 1.85
Cl_max_clean = 1.40
D_prop = 1.75              # m
N_prop = 2400.0            # RPM (constant speed propeller)

# Engine Operating Speed
N_ICE_rated = 3800.0       # RPM (Engine crankshaft speed at rated power)

# ==============================================================================
# ELECTRIC PROPULSION & BATTERY CONSTANTS
# ==============================================================================
specific_power_motor = 4.0      # kW/kg (Continuous rated, high-performance PMSM motor)
specific_power_inverter = 12.0  # kW/kg (High-voltage SiC inverter)
specific_energy_battery = 200.0 # Wh/kg (Pack-level Li-ion / NMC cells with BMS & casing)
sp_power_DCDC = 0.50            # kW/kg (Mil-COTS isolated high-voltage aerospace standard)

# Overhead factors
k_bat_pack = 1.18               # 18% overhead for module casing, BMS, busbars, cooling
k_fuel_sys = 1.05               # 5% overhead for wing bladders, pumps, baffles, unusable trapped fuel

# Battery Safe Operations
C_discharge_continuous_safe = 2.0  # 1/h
C_charge_continuous_safe = 1.0     # 1/h

# Efficiencies
eta_gearbox_ICE = 0.97
eta_gearbox_gen = 0.95
eta_motor = 0.95
eta_inverter = 0.97
eta_discharge = 0.90
eta_charging = 0.95
eta_alternator = 0.80
eta_DCDC_conv = 0.95

# Landing Limits
max_landing_mass_percent = 100.0
max_landing_mass = (max_landing_mass_percent / 100.0) * MTOW

# Atmospheric Baseline Reference
rho_0 = 1.2250  # kg/m^3 (ISA Sea Level)

# ==============================================================================
# GOVERNING FUNCTIONS
# ==============================================================================
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
    Dynamically scales the BSFC polynomial according to the Heywood-scaled
    anchors (BSFC_ICE_opt and BSFC_ICE_max).
    """
    if P_avail_kW <= 0.0 or P_brake_kW <= 0.0:
        return BSFC_ICE_low

    xi = float(np.clip(P_brake_kW / P_avail_kW, 0.05, 1.0))
    
    # Quadratic fit matching xi=0.70 at BSFC_ICE_opt and xi=1.00 at BSFC_ICE_max
    # Coefficients scaled relative to base polynomial:
    rel_opt = BSFC_ICE_opt / BSFC_ICE_opt_base
    a = 222.2 * rel_opt
    b = -311.1 * rel_opt
    c = 333.9 * rel_opt
    
    bsfc = a * (xi ** 2) + b * xi + c
    return float(np.clip(bsfc, BSFC_ICE_opt, BSFC_ICE_low))