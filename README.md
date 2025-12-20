# Arduino Data Logger

A compact Arduino-based data logger that captures environmental and motion data and writes it to an SD card. This project collects sensor readings (barometer, IMU/high-g acceleration) and logs them in a timestamped CSV format for later analysis.

**Key features**
- Logs barometric pressure, temperature (if available), and IMU data (acceleration/gyro) to SD card.
- Simple CSV output for easy import into spreadsheets or data analysis tools.
- Web UI support for live data preview (when enabled).

**Hardware**
- ESP32 Dev Board
- Barometric sensor (supported in `BARO.cpp` / `BARO.h`)
- IMU sensor (supported in `IMU.cpp` / `IMU.h` and `HighG.cpp` / `HighG.h`)
- SD card module (handled by `SDCardLog.cpp` / `SDCardLog.h`)

Wiring will depend on your specific sensors and SD module. Typical connections:
- Power: 3.3V or 5V and GND to sensors and SD module per component requirements
- I2C sensors (IMU/Barometer): connect SDA ,SCL -> corresponding I2C pins on your board
- SD card SPI: MOSI, MISO, SCK, CS pinned to your board's SPI pins

Check each sensor's documentation for voltage and level-shifting requirements.

Software overview
- `Data_Logger.ino` — main Arduino sketch: initializes hardware, coordinates sensor reads, and triggers SD writes.
- `BARO.cpp` / `BARO.h` — barometer sensor support and helper functions.
- `IMU.cpp` / `IMU.h` — inertial measurement unit (IMU) sensor support.
- `HighG.cpp` / `HighG.h` — optional higher-g accelerometer support.
- `SDCardLog.cpp` / `SDCardLog.h` — SD card initialization, file handling, and CSV formatting.
- `WebUI.cpp` / `WebUI.h` — lightweight Web UI / serial output support for live monitoring.
- `DataTypes.h` — shared structs and type definitions used across modules.

Getting started
1. Install the Arduino IDE (or PlatformIO) and set up your board.
2. Connect the sensors and SD module to your board as described above.
3. Open [Data_Logger.ino](Data_Logger.ino) in the Arduino IDE.
4. Adjust any pin definitions or configuration flags at the top of the sketch to match your wiring.
5. Compile and upload to your Arduino board.
6. Insert a FAT32-formatted SD card and check for log files after running.

Log format
- CSV rows with a timestamp and sensor readings. Header and exact column order are defined in `SDCardLog.cpp`.

Configuration tips
- If using I2C sensors, ensure pull-up resistors are present or enabled on the board.
- For reliable SD writes, use quality microSD cards and test formatting to FAT32.
- If using a 3.3V sensor with a 5V Arduino, use level shifters where required.

Development notes
- The codebase is modular: add or replace sensor drivers by following existing `*.cpp`/`*.h` patterns.
- Keep logging frequency reasonable to avoid SD write saturation and missed reads.

Files
- `Data_Logger.ino` — main sketch
- `BARO.*` — barometer driver
- `IMU.*` — IMU driver
- `HighG.*` — high-g accelerometer support
- `SDCardLog.*` — SD logging and CSV file handling
- `WebUI.*` — optional live output/monitoring
- `DataTypes.h` — shared data types

Troubleshooting
- No logs created: verify SD wiring and CS pin configuration, and confirm SD card is FAT32-formatted.
- Sensors not detected: check I2C wiring, addresses, and power rails.
- Corrupted CSV: ensure the board has sufficient power and logging frequency is not too high.

License & contact
- This project is provided as-is. Add a license file if you want to share under a specific license.
- For questions or contributions, open an issue or contact the maintainer.
