# VL53L1X ULD

ST's Ultra Lite Driver for the VL53L1X.

## Provenance

`core/` is ST's code, byte-for-byte as obtained, via
<https://github.com/rneurink/VL53L1X_ULD> (`src/core/`), which redistributes
ST's ULD unchanged:

    core/VL53L1X_api.cpp        core/VL53L1X_api.h
    core/VL53L1X_calibration.cpp  core/VL53L1X_calibration.h
    core/vl53l1_error_codes.h
    platform/vl53l1_types.h

Copyright (c) 2017 STMicroelectronics. Dual licensed: ST Proprietary
(www.st.com/sla0081) **or** BSD 3-clause, at the user's option. Taken here
under BSD 3-clause. The full licence text is in the header of each file.

The files are `.cpp` rather than ST's original `.c` because that is how the
source distribution ships them; the contents are C and are compiled as C++
here, which is why nothing in this project wraps `VL53L1X_api.h` in
`extern "C"` -- the header has no linkage guards, so both sides must agree to
mangle.

## What is *not* upstream

    platform/vl53l1_platform.h
    platform/vl53l1_platform_linux.cpp

ST deliberately leaves the platform layer to the integrator -- upstream's
version is written against Arduino's `Wire.h`. These two files implement the
same eight entry points over Linux `/dev/i2c-N` instead, and are ours.
