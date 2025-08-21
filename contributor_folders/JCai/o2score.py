import numpy as np
import xarray as xr

# -----------------------------------------------------------------------------
# Oxygen and water age from OCIM
# -----------------------------------------------------------------------------
def get_o2_age(lon, lat, depth, method='linear'):
    """
    Get interpolated Oxygen [mmol/(m^3 s)] and water age from OCIM 
    at a given location (lon, lat, depth).

    Parameters
    ----------
    lon, lat, depth : float
        Coordinates of query point
    method : str, optional
        Interpolation method ('linear', 'nearest')

    Returns
    -------
    O2 : float
        Interpolated oxygen
    age : float
        Interpolated water age
    """
    OCIM_nc_file = 'OCIM_model_output.nc'
    ds = xr.open_dataset(OCIM_nc_file)

    # Interpolation
    point = ds.interp(lon=lon, lat=lat, depth=depth, method=method)

    O2 = float(point['O2'].values)
    age = float(point['age'].values)

    return O2, age


# -----------------------------------------------------------------------------
# Oxygen saturation concentration from HYCOM T/S
# -----------------------------------------------------------------------------
def get_o2sat(lon, lat, depth, method='linear'):
    """
    Computes the oxygen saturation concentration at 1 atm total pressure
    in mmol/m^3 given temperature (T, in °C) and salinity (S, in PSU).

    Based on Garcia and Gordon (1992), Limnology and Oceanography.

    Parameters
    ----------
    lon, lat, depth : float
        Coordinates of query point
    method : str, optional
        Interpolation method ('linear', 'nearest')

    Returns
    -------
    o2sat : float
        Oxygen saturation in mmol/m^3
    """
    HYCOM_nc_file = 'HYCOM_model_output.nc'  # or URL if remote
    ds = xr.open_dataset(HYCOM_nc_file)

    # Interpolation
    point = ds.interp(lon=lon, lat=lat, depth=depth, method=method)

    T = float(point['temperature'].values)  # degC
    S = float(point['salinity'].values)     # PSU

    # Coefficients
    A0, A1, A2, A3, A4, A5 = 2.00907, 3.22014, 4.05010, 4.94457, -2.56847e-1, 3.88767
    B0, B1, B2, B3 = -6.24523e-3, -7.37614e-3, -1.03410e-2, -8.17083e-3
    C0 = -4.88682e-7

    TT = 298.15 - T
    TK = 273.15 + T
    TS = np.log(TT / TK)

    # Polynomial terms
    TS2, TS3, TS4, TS5 = TS**2, TS**3, TS**4, TS**5

    # Oxygen solubility (ml/L)
    CO = (A0 + A1*TS + A2*TS2 + A3*TS3 + A4*TS4 + A5*TS5
          + S*(B0 + B1*TS + B2*TS2 + B3*TS3)
          + C0*(S**2))

    o2sat = np.exp(CO)

    # Convert from ml/L to mmol/m^3
    o2sat = (o2sat / 22391.6) * 1e6

    return o2sat


# -----------------------------------------------------------------------------
# Score based on O2 saturation gradient
# -----------------------------------------------------------------------------
def score_o2sat(cc, pc):
    """
    Compute normalized oxygen saturation gradient scores.

    Parameters
    ----------
    cc : tuple
        Current coords (lon, lat, depth)
    pc : list of tuples
        Potential coords [(lon1, lat1, depth1), (lon2, lat2, depth2), ...]

    Returns
    -------
    scores : np.ndarray
        Normalized gradient scores (0-1), length = len(pc)
    """
    # Current location O2sat
    O2sat_cc = get_o2sat(*cc)

    # Potential locations O2sat
    O2sat_pc = np.array([get_o2sat(*loc) for loc in pc])

    grad = np.abs(O2sat_cc - O2sat_pc)

    # Normalize to 0–1
    total = np.sum(grad)
    scores = grad / total if total > 0 else np.zeros_like(grad)

    return scores


# -----------------------------------------------------------------------------
# Example usage (only runs if you execute this file directly)
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    cc = (-120.5, 35.2, 500.0)
    pc = [(-120.4, 35.3, 500.0),
          (-120.6, 35.1, 400.0)]

    scores = score_o2sat(cc, pc)
    print("Scores:", scores)
