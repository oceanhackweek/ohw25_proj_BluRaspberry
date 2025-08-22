import xarray as xr
import numpy as np

# OCIM_nc_file = '../../data/ds_OCIM_struct.nc' # in 'data' folder, update the nc file if needed
# ds_OCIM = xr.open_dataset(OCIM_nc_file)

nemo_nc_file = '../../data/copernicus-subset-jul-01-03-2025.nc' # in 'data' folder, update the nc file if needed
ds_nemo = xr.open_dataset(nemo_nc_file)

# -----------------------------------------------------------------------------
# Oxygen and water age from OCIM
# -----------------------------------------------------------------------------
# def get_o2_age(lon, lat, depth):
#     """
#     get Oxygen [mmol/(m^3 s)] and water age from OCIM at a giving location
#     """
#     point = ds_OCIM.sel(longitude=lon, latitude=lat, depth=depth, method='nearest')
    
#     O2 = float(point['dO_conc'].values)
#     age = float(point['water_age_sec'].values)
    
#     return O2, age

# -----------------------------------------------------------------------------
# Oxygen saturation concentration from HYCOM T/S
# -----------------------------------------------------------------------------
def get_o2sat(lon, lat, depth):
    """
    Computes the oxygen saturation concentration at 1 atm total pressure
    in mol/m^3 given temperature (T, in °C) and salinity (S, in ‰).

    From Garcia and Gordon (1992), Limnology and Oceanography.
    Note: The "A3*TS^2" term in the paper is incorrect.

    Valid range:
        T(freezing) <= T <= 40 °C
        0 ‰ <= S <= 42 ‰

    Check value:
        T = 10.0 °C, S = 35.0 ‰ → o2sat = 0.282015 mol/m^3 = 282 mmol/m^3
    """

    # --------------------------------------------------------------------------
    # grab T and S from location (lon, lat, depth) 
    # --------------------------------------------------------------------------

    point = ds_nemo.sel(longitude=lon, latitude=lat, depth=depth, method='nearest')

    T = float(point['thetao'].values) # potential temperature, unit of deg c
    S = float(point['so'].values) # pratical salinity, unit of PSU
    

    # -------------------------------------------------------------------------
    # calculate the satuation rate
    # --------------------------------------------------------------------------
    
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
    o2sat = (o2sat / 22391.6) * 1000.0 * 1000.0 

    return o2sat

# -----------------------------------------------------------------------------
# Score based on O2 saturation gradient
# -----------------------------------------------------------------------------
def score_o2sat(cc, pc):
    """
    input:
    ---------------------------
    cc: tuple
        current coords (lon0, lat0, depth0)
    pc: list of tuples
        potential coords [(lon1, lat1, depth1), (lon2,lat2, depth2), ....]

    get O2sat_{} based on the lon, lat, depth.
    
    gradient = np.abs(O2sat_cc - O2sat_pc)

    normalized to 0-1 as score

    return:
    ---------------------------
    normalized gradient scores (0-1), with the size of pc
    
    """
    # AOR in current location
    O2sat_cc = get_o2sat(*cc)

    # AOR in potential locations
    O2sat_pc = np.array([get_o2sat(*loc) for loc in pc])

    grad = np.abs(O2sat_cc - O2sat_pc)

    # Create mask of valid (non-NaN) entries
    valid_mask = ~np.isnan(grad)

    # Initialize scores array with NaNs
    scores = np.full_like(grad, np.nan, dtype=float)

    # Normalize only non-NaN gradients
    total = np.sum(grad[valid_mask])
    if total > 0:
        scores[valid_mask] = grad[valid_mask] / total
    # else: all zeros remain NaN

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

# -----------------------------------------------------------------------------
# U and V from NEMO
# -----------------------------------------------------------------------------

def get_uv(lon, lat, depth):
    """
    get geostrophic velocity u and v [m/s] from Global Ocean Physics Analysis and Forecast (NEMO model) at a giving location
    """

    # --------------------------------------------------------------------------
    # grab u and v from location (lon, lat, depth) 
    # --------------------------------------------------------------------------

    point = ds.sel(longitude=lon, latitude=lat, depth=depth, method='nearest')

    u = float(point['uo'].values) # eastward ocean current velocity
    v = float(point['vo'].values) # northward ocean current velocity

    return u,v