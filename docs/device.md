# microSD device facts (MARV V2 Rev 1)

Copied from the MARV V2 firmware repository's `docs/devices.md` and `docs/pinout.md`. Items marked UNKNOWN or "unverified" have not been checked against the datasheet or on hardware. "Measured" means measured on this board.

## microSD (Molex 47219-2001 socket J11, native 4-bit SD)

Pins: see "Pins and bus" below.

- Bus: PIO + DMA native 4-bit SD, not SPI mode. 3.3 V signalling only.
- CMD and DAT0-3 have external 10k pull-ups; CLK has none and is driven through a 22 ohm series resistor.
- The socket has no card-detect switch and no GPIO is used for detect; firmware has to probe for the card.
- Measured, card on the bench: SanDisk 64 GB SDXC (CID manufacturer 0x03, OEM "SD", product "SK64G",
  revision 8.0); OCR 0xC0FF8000 (high capacity, 3.2-3.4 V); CSD version 2, 124735488 blocks of 512 bytes;
  ACMD41 ready after about 14 ms; busy after a single-block write about 1 ms.

## Pins and bus (from pinout.md)

All GPIO are 3.3 V logic and are not 5 V tolerant.

| GPIO | Net | Function used | Status |
|---|---|---|---|
| 28 | SD_DAT0 | PIO native SD DAT0 | microSD J11; 10k pull-up (R36-R40 cover CMD and DAT0-3) |
| 29 | SD_DAT1 | PIO native SD DAT1 | microSD J11; 10k pull-up (R36-R40) |
| 30 | SD_DAT2 | PIO native SD DAT2 | microSD J11; 10k pull-up (R36-R40) |
| 31 | SD_DAT3 | PIO native SD DAT3 | microSD J11; 10k pull-up (R36-R40) |
| 32 | SD_CLK_MCU | PIO native SD clock | through 22 ohm R58 to SD_CLK; no pull-up |
| 33 | SD_CMD | PIO native SD command | microSD J11; 10k pull-up (R36-R40) |

| Peripheral | GPIOs | Device |
|---|---|---|
| Native 4-bit SD, PIO + DMA | DAT0-DAT3 28-31, CLK 32, CMD 33 | microSD J11 only |
| Card detect | none | the J11 socket has no detect switch |

| Item | Value |
|---|---|
| microSD | 10k pull-ups to V3V3_SYS on CMD and DAT0-3 (R36-R40); none on CLK; GPIO32 reaches SD_CLK through 22 ohm R58 |

Rails: V3V3_SYS powers the microSD and its pull-ups.

Constraints:

- SD DAT0-DAT3 are consecutive in ascending GPIO order (a PIO `in pins` requirement). All six SD signals use GPIO28-33, in the GPIO16-47 PIO window. The LED and ESC PIO signals need the same window, so every PIO user shares GPIOBASE = 16.
- There is no SD card-detect.

## Datasheet

| Device | URL |
|---|---|
| Molex 47219-2001 microSD socket | https://www.es.co.th/Schemetic/PDF/472192001.PDF |
