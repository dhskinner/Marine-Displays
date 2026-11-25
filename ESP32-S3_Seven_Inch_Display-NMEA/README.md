Boating with the Baileys
Apr 2026

ESP32 customisable display using data directly from the boats NMEA2000 network. 
------------------------------

It features:
- 95 N2K data fields supported — engine, transmission, fluid levels, rudder, heading, rate of turn, attitude, battery/DC, boat speed, depth, distance log, GPS position, COG/SOG, XTE, waypoints, AIS, wind, environment/temp/pressure, trim tabs, system time
- Customisable background images,
- Custom icons, 
- Alerting through icon colours
- WebUI driven
- Touch and non Touch options
- Number display with different display options, Single large display, Dual screen, Quad Screen
- Anchor watch display
- Custom per-screen display labels (override the default field description shown on screen)
- Dual Gauge, Guage with added number display (Wind instrument style)
- Graph display with the option for an additional data source & data retained while device is powered
- Navigation Display with heading & Dual data fields
- Position and Time display
- AIS Radar display
- Dimming with night mode, display sleep options
- Unit conversions
- OTA updates
- NMEA 2000 over WiFi from a gateway — see below

### NMEA 2000 over WiFi

Instead of wiring the display to the CAN bus, it can take the same NMEA 2000 data from a WiFi gateway. In the display's web page go to **Network Setup → NMEA 2000 data source**, choose **WiFi gateway** and enter:

- **Gateway format**: YD RAW (Yacht Devices YDWG-02 / YDEN-02), Actisense NGT binary, or SeaSmart `$PCDIN` (Digital Yacht, Chetco and others). Use whatever your gateway is set to send.
- **Connection**: TCP (the display connects to the gateway's IP address and port) or UDP (the gateway sends to the display on a port).
- **Gateway IP address and port**, as shown in the gateway's own settings.

The display and gateway must be on the same WiFi network (your boat's router, or the gateway's own hotspot), set at the top of the same page. One **Save & Reboot** saves both. The display only receives: it never sends anything to the gateway or onto the NMEA 2000 bus. The Network Setup page shows whether it's connected and how many messages it has received. If it shows "unreadable lines", the format setting doesn't match the gateway.

Please report what works (and what doesn't) with your gateway on the [GitHub issues page](https://github.com/Boatingwiththebaileys/Marine-Displays/issues).

ESP32-S3 7 inch Display

The hardware used:
Waveshare ESP32-S3-Touch-LCD-7: 7 inch 800×480 capacitive touch display, ESP32-S3 (N16R8: 16MB flash, 8MB PSRAM), CAN and RS485 interfaces https://www.waveshare.com/esp32-s3-touch-lcd-7.htm

The screens use the same 480×480 layout as the 4 inch display, drawn in the middle of the 800×480 panel with black bars either side.

Differences from the 4 inch board:
- No buzzer: alerts are shown with icon colours only.
- The backlight is on/off only (no dimming).
- No real-time clock: the time comes from Signal K / NMEA 2000 and is kept while the display is powered.
- No motion sensor, so the attitude display has no data.

Build and upload instructions:
1. Install PlatformIO and required toolchains.
2. Set the board's **UART selection** switch to **UART1** and connect the USB-C port marked **UART** (not the one marked USB). The USB port shares its pins with CAN, so it switches off when the firmware starts NMEA 2000.
3. Set `upload_port` / `monitor_port` in `platformio.ini` to that port's COM port.
4. Open this folder and run `pio run`, then `pio run --target upload`.

`pio run --target upload` writes the bootloader and partition table as well as the app, so a board straight from Waveshare needs nothing extra. (An upload with `--target nobuild` writes only the app, and a new board then restarts endlessly until a full upload is done.)

Use an SD card for your icons and images. Store icons and PNGs (ideally monochrome images) so you can recolor them in the UI. Convert larger background images to `.bin` files. There is a conversion script in the project to help with this.

The screen layout is 480x480, so make your background images this size.

Use 70x70 for icons.

I used Figma to create my images.

Save them on the SD card in a folder called `/assets`.

On first boot, the display creates an SSID called ESP32-7inchDisplay with the password: 12345678 - Browse to 192.168.4.1 to configure your display. 

To use it on the NMEA2k network, connect the CAN H/L terminals to your network and the display should show your data. Make sure the device is powered from the same source as the N2K network, remember termination (the board has a CAN terminal resistor switch) and also placement of the display is important. Do so at your own risk.

The Baileys



How to convert PNG backgrounds
------------------------------

This project uses raw RGB565 `.bin` background files on the SD card. The workflow we used is:

- Convert `*.png` to RGB565 `.bin` using `convert_png_to_rgb565.py` (or run `batch_convert.sh` which calls it for the assets).
- Copy the produced `.bin` files to the SD card `assets/` folder on the display.

Example (from project root):

```bash
# convert a single PNG to a .bin
python3 convert_png_to_rgb565.py assets/Rev_Counter.png assets/Rev_Counter.bin

# or run the batch helper (installs Pillow if needed)
./batch_convert.sh
```
