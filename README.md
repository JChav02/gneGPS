# GNE's Car Mounted GPS Unit (BETA BRANCH)

## What's this?

This is a modular, portable, car mounted solution meant to provide GPS functionality to vehicles which do not have it, have lost it or need a replacement. It takes the readings from an [uBlox GY-GPS6MV2](https://www.epitran.it/ebayDrive/datasheet/NEO6MV2.pdf) and shows them on TM1637 displays.

## Features

- Speed
- Altitude
- Latitude & longitude
- Time
- Compass
- Total distance traveled

## Dependencies

- [NeoGPS](https://github.com/SlashDevin/NeoGPS) by SlashDevin: A NMEA and ublox GPS parser for Arduino, configurable to use as few as 10 bytes of RAM.
- [TM1637 Library](https://www.arduino.cc/reference/en/libraries/tm1637/): Driver for 4 digit 7-segment display modules, based on the TM1637 chip.