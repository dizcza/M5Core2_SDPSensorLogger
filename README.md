# SDP sensor reader in Arduino

Read SDP sensor differential pressure in Arduino and save the measurements to an SD card. Log the progress to the M5 Lcd screen and a txt log file.

Board: M5Core2.

The project is ported from https://github.com/dizcza/esp32-sdpsensor.

You need to install the [`SDPSensors.h`](https://github.com/UT2UH/SDP3x-Arduino/tree/SDP8x) Arduino lib.

The implementation is robust to SD card failures: don't loose any sensor measurment.

