"""SITL smoke test — validates MSP wire protocol against a real Betaflight
binary running in Docker.

Skipped unless `SITL_TCP=host:port` env var is set, so it never breaks local
dev or main CI. To run:

    cd sitl && docker compose up -d
    cd ../companion && SITL_TCP=localhost:5761 python -m unittest tests.test_sitl_smoke -v

The test does NOT validate flight dynamics — just the protocol layer:
- MSP_API_VERSION returns a sensible response
- MSP_FC_VARIANT identifies as BTFL
- MSP_SET_RAW_RC is accepted without error
- MSP_RC reads back values that include what we just wrote
"""
import os
import time
import unittest

from racer_companion import msp


@unittest.skipUnless(os.environ.get("SITL_TCP"), "set SITL_TCP=host:port to run")
class TestSitlSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.endpoint = os.environ["SITL_TCP"]
        cls.client = msp.MspClient(f"tcp://{cls.endpoint}")
        # Drain any boot junk
        time.sleep(0.5)
        cls.client.poll()

    @classmethod
    def tearDownClass(cls):
        cls.client.close()

    def _request_and_collect(self, cmd: int, timeout_s: float = 1.0):
        self.client.send(cmd)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for c, p in self.client.poll():
                if c == cmd:
                    return p
            time.sleep(0.02)
        self.fail(f"No MSP response for cmd={cmd} within {timeout_s}s")

    def test_api_version(self):
        payload = self._request_and_collect(msp.MSP_API_VERSION)
        self.assertGreaterEqual(len(payload), 3, msg=f"got {payload.hex()}")

    def test_fc_variant(self):
        payload = self._request_and_collect(msp.MSP_FC_VARIANT)
        self.assertEqual(payload[:4], b"BTFL", msg=f"got {payload!r}")

    def test_set_raw_rc_round_trip(self):
        # Write known values, read back via MSP_RC.
        target = [1234, 1456, 1678, 1100, 1500, 1500, 1900, 1500]
        for _ in range(5):
            self.client.send_raw_rc(target)
            time.sleep(0.05)

        payload = self._request_and_collect(msp.MSP_RC, timeout_s=2.0)
        rc = msp.decode_rc(payload)
        # The first 4 channels (R/P/Y/T) should reflect what we sent if
        # msp_override_channels_mask is correctly set to 15. AUX channels
        # may be controlled by SITL defaults, so don't assert on those.
        for i, expected in enumerate(target[:4]):
            self.assertAlmostEqual(rc[i], expected, delta=20,
                                   msg=f"channel {i}: sent {expected}, got {rc[i]}")


if __name__ == "__main__":
    unittest.main()
