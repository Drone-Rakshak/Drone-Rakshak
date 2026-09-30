import os


BACKEND_URL = os.getenv(
    "BACKEND_URL",
    "http://127.0.0.1:8000"
)

TELEMETRY_ENDPOINT = os.getenv(
    "TELEMETRY_ENDPOINT",
    "/api/telemetry/"
)

UAV_ID = os.getenv(
    "UAV_ID",
    "DRONE-001"
)
