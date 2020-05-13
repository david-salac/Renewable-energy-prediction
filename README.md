# Renewable energy prediction
Helpful notebook for the prediction of energy from renewable resources.

Author: David Salac <http://www.github.com/david-salac>

## Solar photovoltaic power plants prediction
There are many ways how to predict the production of photovoltaic installations. The represented one uses the PVLIB library (that is publicly available under a commercially friendly license).

### What are the inputs
The list of inputs follows. The vendor provides some of them, some of them are dependant on location of installation, and remaining are dependant on the time of prediction.

#### Environmental variables
Weather conditions and current time define them.

1. _E<sub>GHI</sub>_: acronym of the Global Horizontal Irradiance, units: W⋅m<sup>-2</sup>

2. _T_: temperature, units: &deg;C

3. _R_: ground albedo (reflectance of the ground around), units: unitless (value from 0 to 1)

4. _t_: calendar time with precision up to seconds, typically in UTC timezone.

#### Installation dependant parameters
Positional (depends on spatial solution):

1. _A<sub>t</sub>_: tilt angle, units: degree (&deg;)

2. _A<sub>a</sub>_: azimuth angle, units: degree (&deg;)

3. _P<sub>lon, lat</sub>_: longitude and latitude of the location, units: degree (&deg;)

Defined by vendor:

1. _G<sub>s</sub>_: linear coefficient of efficiency decrease when 1&deg;C temperature increase happens, units: unitless

2. _T<sub>opt</sub>_: operational temerature of installation, units: &deg;C

3. _E<sub>std</sub>_: standard irradiance, units: W⋅m<sup>-2</sup>

4. _T<sub>std</sub>_: standard temperature, units: &deg;C

5. _P_: maximal possible power to be produced (capacity), units: W

### Prediction logic
How power prediction works is presented in the following example:

```
from typing import Tuple
from datetime import datetime

import pvlib


def solar(
    E_ghi: float, T: float, R: float, t: datetime, *,
    A_t: float, A_a: float, P_lon_lat: Tuple[float, float],
    G_s: float, T_opt: float, E_std: float, T_std: float, P: float
) -> float:
    """
    Predict the photovoltaic power production.

    :param E_ghi: global horizontal irradiance
    :param T: temperature
    :param R: ground albedo (reflectance of the ground around)
    :param t: calendar time with precision up to seconds, in UTC
    :param A_t: tilt angle
    :param A_a: azimuth angle
    :param P_lon_lat: longitude and latitude of the location
    :param G_s: linear coefficient of efficiency decrease
    :param T_opt: operational temerature of installation
    :param E_std: standard irradiance
    :param T_std: standard temperature
    :param P: capacity of installation

    :return: The power prediction of installation in Watts.
    """
    # Compute the Solar Zenith Angle and Solar Azimuth Angle
    sza_saa = pvlib.solarposition.get_solarposition(t,
                                                    P_lon_lat[1], P_lon_lat[0])
    # Extract SZA and SAA
    A_sza, A_saa = sza_saa['zenith'], sza_saa['azimuth']
    # Compute DHI and DNI (Direct Normal Irradiance and
    # Diffuse Horizontal Irradiance) using Erbs model
    dhi_dni = pvlib.irradiance.erbs(E_ghi, A_sza, t)
    # Extract DHI and DNI
    E_dhi, E_dni = dhi_dni['dhi'], dhi_dni['dni']
    # Computes the total irradiance
    irr = pvlib.irradiance.get_total_irradiance(A_t, A_a, A_sza, A_saa,
                                                E_dni, E_ghi, E_dhi, albedo=R)
    # Extract total irradiance
    E_irr = irr['poa_global'].to_numpy().squeeze()

    # Normalized temperature
    T_n = T + E_irr * (T_opt - 20.0) / 800.0

    # Predict the power:
    return (E_irr / E_std) * (1.0 + G_s) * (T_n - T_std) * P


# EXAMPLE
power = solar(E_ghi=800.0, T=23.5, R=0.25, t=datetime(2020, 5, 12, 10, 30),
              A_t=30.0, A_a=30.0, P_lon_lat=(-0.12795, 51.50774),
              G_s=-0.004, T_opt=47.0, E_std=1000.0, T_std=43.0, P=1000.0)

```

## Wind turbine power prediction
Wind turbines power production is typically determined as a cube of the wind speed until the saturation. 

A typical wind turbine is regulated by the angle of blades (pitch type). The mapping from wind speed to power production is called a *power curve* and the vendor provides it.

### What are the inputs
There is the weather at a given time and constant related to the transformation of weather data. Also, there are parameters provided by the vendor.

#### Environmental variables
Weather conditions:

1. _v<sub>w</sub>_: wind speed at some referential altitude, units: m⋅s<sup>-1</sup>

2. _D<sub>a</sub>_: air density: kg⋅m<sup>-3</sup>

Constants related to provided data:

1. _A<sub>r</sub>_: referential altitude (to which wind speed is provided), units: m

2. _L_: coefficient for height transformation, used for calibrating of wind speed if it is measured in a different altitude then the height of installation, unit: unitless

#### Installation dependant parameters
Provided by vendor.

1. _H_: height of the installation (from rotor center to ground), units: m

4. _C_(_v_): mapping from the wind speed to percentage of capacity: m⋅s<sup>-1</sup> ⟶ &#37;

5. _P_: maximal possible power to be produced (capacity), units: W

The power curve can also be defined differently, but it should always be possible to remap it to the presented version.

### Prediction logic
How the power prediction works is presented in the following example:

```
from typing import Callable
import numpy as np


def wind(v_w: float, D_a: float, A_r: float, L: float,
         *, H: float, C: Callable[[float], float], P: float) -> float:
    """
    Predict the power production of the wind turbine.

    :param v_w: wind speed
    :param D_a: air density
    :param A_r: referential altitude
    :param L: coefficient for height tranformation
    :param H: height (from ground up to the center of rotor)
    :param C: mapping from wind speed to efficiency
    :param P: capacity of the installation
    :return: power prediction for given conditions.
    """
    # Calibrate wind speed (measured for given altitude) to the height
    v_w *= (H / A_r) ** L

    # Calibrate wind speed to density of air (divided by referential one)
    # referential density is 1.225 based on the value defined by
    # International Standard Atmosphere organization
    v_w *= (D_a / 1.225) ** (1.0 / 3.0)
    return P * C(v_w)


# EXAMPLE
# 1) define power curve:
def power_curve(v_s) -> float:
    # Provided by vendor
    curve = {2.0: 0.0, 3.0: 0.0, 4.0: 0.02727,
             5.0: 0.7208, 6.0: 0.13368, 7.0: 0.21711,
             8.0: 0.32513, 9.0: 0.45454, 10.0: 0.5925,
             11.0: 0.7176, 12.0: 0.8117, 13.0: 0.8673,
             14.0: 0.8941, 15.0: 0.9048, 16.0: 0.9080,
             17.0: 0.9090, 18.0: 0.9090, 19.0: 0.9090,
             20.0: 0.9090, 21.0: 0.9090, 22.0: 0.9090,
             23.0: 0.9090, 24.0: 0.9090, 25.0: 0.9090}
    c_w = np.fromiter(curve.keys(), dtype=float)
    c_p = np.fromiter(curve.values(), dtype=float)
    # linear interpolation + extrapolates to zeros elsewhere
    return np.interp(v_s, c_w, c_p, left=0.0, right=0.0)


# 2) compute power:
power = wind(v_w=10.0, D_a=1.250, A_r=10.0, L=0.1431,
             H=30, C=power_curve, P=1250000)

```