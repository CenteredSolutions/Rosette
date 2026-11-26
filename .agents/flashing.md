# Flashing an RP2040 or RP2350 board

Use `tools/pico_uf2_flash.py` ([GETTING_STARTED.md](../GETTING_STARTED.md), section 5). It opens the console at
1200 baud, which reboots a running Pico SDK image into BOOTSEL, then copies the UF2 to the drive that appears. A
1200-baud reboot keeps power on, so external chips keep their state, which is useful when reproducing a fault.

## Without a BOOTSEL drive

On a headless host or over SSH, nothing mounts the BOOTSEL drive, and polkit may refuse `udisksctl mount`. Use
picotool over libusb instead.

- The picotool that the SDK fetches into `<build>/_deps` is built without libusb. Build one with
  `libusb-1.0-0-dev` installed:
  `cmake -S <build>/_deps/picotool-src -B build/picotool -DPICO_SDK_PATH=<build>/_deps/pico_sdk-src`.
- In a container, give it the bus: `--privileged --user 0 -v /dev/bus/usb:/dev/bus/usb`.
- Device-selection flags go after the file name: `picotool load -x app.uf2 --bus B --address A`.
- The board's USB address changes when it reboots, so `load -f` loses it. Run `picotool reboot -u -f --bus B
  --address A`, find the BOOTSEL device again (`lsusb -d 2e8a:`), then `load -x`.
- If a later picotool session fails with "unknown error" while the first succeeded, reset the USB port
  (`USBDEVFS_RESET`) before each call.

Both chips report the name "Raspberry Pi Pico", so tell them apart by USB product id (vendor `2e8a`):

| Chip | Application (USB stdio) | BOOTSEL |
| --- | --- | --- |
| RP2040 | `000a` | `0003` |
| RP2350 | `0009` | `000f` |

A blank board, or one whose image has no USB stdio, needs BOOTSEL held at power-up.
