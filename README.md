# Parallel Hybrid-Electric MALE UAV Mission Simulation

Time-marching 11-phase point-mass flight dynamics and powertrain simulation for a **1,000 kg tactical MALE UAV** powered by a **Parallel Hybrid-Electric powertrain** (**50.0 kW CI engine + 55.0 kW direct-drive axial-flux PMSM** with a **40.0 kWh Li-ion battery pack**).

## Key Specifications
* **Airframe:** 1,000.0 kg MTOW | 18.0 m wingspan ($AR = 14.73$) | $22.0\text{ m}^2$ wing area | $C_{D0} = 0.0220$ | $(L/D)_{\max} = 19.13$
* **Powerplant:** 50.0 kW CI engine (98.47 kg) + planetary reduction gearbox & overrunning sprag clutch (16.00 kg, $\eta_{\text{gb}} = 0.97$) + coaxial 55.0 kW axial-flux PMSM & inverter (19.33 kg)
* **Energy Storage:** 40.0 kWh NMC Li-ion pack (236.00 kg, 200 Wh/kg, 2C discharge / 1C charge) + 85.42 kg usable Jet-A1 (79.87 kg mission + 5.55 kg reserve)
* **Energy Management Strategy:**
  * **Takeoff, Pull-Up & Initial Climb:** 100% ICE + 100% EM boost.
  * **Cruise-Climb (1,500–3,000 m):** 100% ICE only ($\text{DoH} = 0$).
  * **Outbound Cruise:** ICE propulsion + generator recharge up to 90% SoC.
  * **On-Station Loiter:** 3-stage pure-electric stealth protocol (**15 km silent ingress**, **silent stare over target**, **15 km silent egress** with ICE decoupled via sprag clutch) down to 20% SoC, followed by mid-air relight and sustained ICE standoff loiter.

## Quick Start

### 1. Install Dependencies
Requires Python 3.8+ with `ambiance` (ICAO 1993 Standard Atmosphere) and `matplotlib`:
```bash
pip install ambiance numpy matplotlib
```

### 2. Run the Baseline Simulation
Execute your main mission runner script from the repository root:
```bash
python main.py
```
*(Replace `main.py` with your top-level entry script filename if different.)*

### 3. Plug-and-Play Custom Inputs
* **`jatayu_uav.py`**: Airframe mass/aerodynamics, downsized ICE rating, PMSM/inverter ratings, and component efficiencies.
* **`mission_settings.py`**: Battery capacity, C-rate limits, $\text{SoC}_{\min}/\text{SoC}_{\max}$ bounds, standoff gate distance (15 km), and transit distances.
* **`Phases/*.py`**: Phase-level power splits (`percent_ICE_power`, `percent_electric_power`).

## Baseline Benchmark Results
| Metric | Simulated Value |
| :--- | :--- |
| Takeoff Ground Roll / 15 m Obstacle | 205.79 m / 339.24 m |
| Initial Climb Rate (at 1,500 m) | **5.42 m/s** |
| Pure-Electric Silent Stealth Window | **50.8 min** (7.6 min ingress, 35.5 min stare, 7.6 min egress) |
| On-Station Loiter Endurance | **11.27 h** (0.85 h silent + 10.43 h ICE) |
| Total Mission Endurance | **13.63 h** |
| Mission Fuel Consumed / Final SoC | 79.26 kg Jet-A1 / 20.00% SoC |
| Touchdown Gross Mass / Landing Roll | 920.74 kg / 106.67 m |
