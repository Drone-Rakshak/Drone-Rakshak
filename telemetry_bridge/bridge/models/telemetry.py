from dataclasses import dataclass
from typing import Optional


@dataclass
class TelemetryData:

    uav_id: str
    timestamp: float

    latitude: Optional[float] = None
    longitude: Optional[float] = None
    altitude: Optional[float] = None

    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None

    battery: Optional[float] = None

    armed: Optional[bool] = None
    flight_mode: Optional[str] = None
