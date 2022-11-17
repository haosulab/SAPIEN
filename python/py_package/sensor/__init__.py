from .activelight import ActiveLightSensor

try:
    from .activelight import ActiveLightSensorCUDA
except ImportError:
    pass
