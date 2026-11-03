"""Opt-in interface contract, not a configured cloud or local model integration.

No HTTP client, model launch, credentials, or automatic image upload is present.
A future transport must enforce its own network deadline. Injected test
transports are always marked simulated; their results are never model evidence.
"""
import math
import time


class VLMBridge:
    def __init__(self, enabled=False, transport=None, timeout_seconds=5.0):
        if not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
            raise ValueError('invalid VLM timeout')
        self.enabled, self.transport, self.timeout = enabled, transport, timeout_seconds

    def describe(self, target_id, jpeg):
        if not self.enabled:
            return dict(status='disabled', simulated=False, hardware_write=False)
        if self.transport is None:
            return dict(status='not_configured', simulated=False, hardware_write=False)
        if not isinstance(target_id, str) or not target_id or not isinstance(jpeg, bytes) or not jpeg:
            return dict(status='invalid_input', simulated=True, hardware_write=False)
        started = time.monotonic()
        try:
            result = self.transport(dict(task='describe_registered_object', target_id=target_id,
                                         image_jpeg=jpeg), timeout_seconds=self.timeout)
            if time.monotonic() - started > self.timeout:
                raise TimeoutError()
            if (not isinstance(result, dict) or set(result) != {'description', 'attributes'}
                    or not isinstance(result['description'], str) or not 0 < len(result['description']) <= 1000
                    or not isinstance(result['attributes'], dict)
                    or len(result['attributes']) > 12
                    or not all(isinstance(k, str) and isinstance(v, str) and len(k) <= 80 and len(v) <= 200
                               for k, v in result['attributes'].items())):
                return dict(status='invalid_response', simulated=True, hardware_write=False)
            return dict(status='ok', simulated=True, target_id=target_id,
                        description=result['description'], attributes=result['attributes'],
                        instance_identity_verified=False, hardware_write=False)
        except TimeoutError:
            return dict(status='timeout', simulated=True, hardware_write=False)
        except Exception:
            return dict(status='failed', simulated=True, hardware_write=False)
