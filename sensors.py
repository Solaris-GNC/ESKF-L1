from dataclasses import dataclass
import numpy as np


@dataclass
class Sensor():
    """Latest measurement of an aiding sensor, consumed by ESKF.run.

    Whether a measurement is new is decided by its timestamp, not its value, so two
    consecutive equal readings are both used.

    Attributes:
        kind: Sensor name; ESKF.run calls ESKF.update_<kind> ("mag", "accel", "baro", ...).
        ts: Timestamp of the latest measurement (s).
        data: Latest measurement, passed as the first argument of update_<kind>.
        sigma: Measurement noise std, passed as the second argument of update_<kind>.
        last_ts: Timestamp of the last measurement used by the filter (s).
        flag: Unused.
    """
    kind: str
    ts: float
    data: np.ndarray
    sigma: float | np.ndarray
    last_ts: float
    flag: bool = False

    def new_meas(self, ts, data):
        """Store a new measurement.

        Args:
            ts: Timestamp (s).
            data: Measurement.
        """
        self.ts, self.data = ts, data

    def has_new_data(self):
        """Whether the stored measurement has not been used by the filter yet.

        Returns:
            True if ts > last_ts.
        """
        return self.ts > self.last_ts
