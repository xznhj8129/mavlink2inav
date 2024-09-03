# Mavlink2INAV
Python based MAVLINK control for INAV UAVs

# NOT READY YET #
https://asciiflow.com/#/

## Operating modes:
#### Override
Activated with MSP RC OVERRIDE over normal flight receiver

Wiring:
* RC RX to RX port UART
* Flight Computer to MSP port UART

#### MSP-RC
Full control of UAV as an MSP receiver

Wiring:
* Telemetry radio to Flight Computer
* Flight computer to MSP port UART

#### MAVLINK-RC
Full control of UAV as a MAVLINK receiver

Wiring:
* Telemetry radio to Flight Computer
* Flight Computer to MAVLINK telemetry port UART