"""Digital shadow of a signalized intersection (Rusia site).

Root package of the system. The flow is one-directional physical→virtual
(monitoring without actuation): it is NOT a digital twin, there is no traffic
signal control. It grows in phases (perception, calibration, SUMO
synchronization, visualization); this package fixes the common import
convention.
"""

__version__ = "0.1.0"
