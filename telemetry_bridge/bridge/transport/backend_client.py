import requests

from bridge.config import (
    BACKEND_URL,
    TELEMETRY_ENDPOINT,
)


class BackendClient:

    def __init__(self):
        self.url = (
            BACKEND_URL +
            TELEMETRY_ENDPOINT
        )

    def send_telemetry(self, telemetry: dict):

        try:

            response = requests.post(
                self.url,
                json=telemetry,
                timeout=5,
            )

            response.raise_for_status()

            print(
                f"[BACKEND] Telemetry accepted | "
                f"UAV={telemetry['uav_id']}"
            )

            return True

        except requests.RequestException as exc:

            print(
                f"[BACKEND ERROR] {exc}"
            )

            return False
