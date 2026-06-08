from math import pi, log

# Constants 
decomposition_rate = 0.1 # deconpostion rate constant - how fast the bloom breaks down over time
reference_temperature = 20.0 # Celsius

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
