from math import pi, log

# Constants 
decomposition_rate = 0.1 # deconpostion rate constant - how fast the bloom breaks down over time
reference_temperature = 20.0 # Celsius
M_harmful_threshold = 500.0 # tunable parameter

# Bloom physical constants (assumptions)
BLOOM_DEPTH = 1.0 # metres, thickness
BLOOM_DENSITY = 1.0 # g/m^3

# Coordinates - North Sea coastal waters new The Hague
LATITUDE = 52.0
LONGITUDE = 4.2

import openmeteo_requests
import pandas as pd
import requests_cache
from retry_requests import retry

cache_session = requests_cache.CachedSession('.cache', expire_after = 3600)
retry_session = retry(cache_session, retries = 5, backoff_factor = 0.2)
openmeteo = openmeteo_requests.Client(session = retry_session)

url = "https://marine-api.open-meteo.com/v1/marine"
params = {
	"latitude": 52,
	"longitude": 4.2,
	"hourly": "sea_surface_temperature",
}
responses = openmeteo.weather_api(url, params = params)

response = responses[0]

hourly = response.Hourly()
sea_surface_temperature = hourly.Variables(0).ValuesAsNumpy()

hourly_data = {
	"date": pd.date_range(
		start = pd.to_datetime(hourly.Time(), unit = "s", utc = True),
		end =  pd.to_datetime(hourly.TimeEnd(), unit = "s", utc = True),
		freq = pd.Timedelta(seconds = hourly.Interval()),
		inclusive = "left"
	)
}

hourly_data["sea_surface_temperature"] = sea_surface_temperature

hourly_dataframe = pd.DataFrame(data = hourly_data)

water_temperature = float(hourly_dataframe[hourly_dataframe["date"].dt.date 
== hourly_dataframe["date"].dt.date.iloc[0]]
["sea_surface_temperature"].mean())

def predict_severity(radius: float, temperature: float):
    # Step 1: calculate area from the algal bloom's radius
    area_algal_bloom = pi * (radius ** 2)

    # Step 2: convert area to mass
    mass_algal_bloom = area_algal_bloom * BLOOM_DEPTH * BLOOM_DENSITY

    # Step 3: check if already harmful
    if mass_algal_bloom < 500.0:
        return "SAFE", None

    # Step 4: calculate days until harmful
    t_harmful = -(reference_temperature / (decomposition_rate * temperature)) * log(1 - M_harmful_threshold / mass_algal_bloom)

    # Step 5: assign severity level
    if t_harmful <= 1:
        severity = "CRITICAL"
    elif t_harmful <= 3:
        severity = "HIGH"
    elif t_harmful <= 7:
        severity = "MODERATE"
    else:
        severity = "LOW"

    return severity, round(t_harmful, 2)

severity, t_harmful = predict_severity(15, water_temperature)
print(f"Risk: {severity}, Days until harmful: {t_harmful}")