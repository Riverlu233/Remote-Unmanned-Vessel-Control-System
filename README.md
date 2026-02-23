# Remote-Unmanned-Vessel-Control-System

## Introduction

This project constitutes a comprehensive simulation initiative, with the research scenario centred upon the navigation of unmanned vessels within ocean currents. Specifically, it addresses how the vessel's heading should be selected to maintain a designated course amidst currents whose magnitude and direction exhibit random distribution. The system design integrates three key aspects: the vessel's autonomous control mechanisms; communication protocols between the vessel and a remote command centre; and the encoding of maritime communication channels alongside the encoding of command signals and their source.

## Project Structure

```text
├── README.md                      # README
├── LICENSE                        # MIT
├── UAV_Path_Planning.pdf          # Final Report
├── src/                           # Source Code
   ├── comm_unit.m
   ├── control_system.m
   ├── info_processing.m
   └── main.m
```

## Getting Started

Directly run MATLAB files main_ver2.m and the programme will generate the final navigation route map, with true north at the top. The parameters used during communication and the navigation instructions can all be modified in this file.

## Results

Please refer to the report for the results.

## Credits & Acknowledgements

The control section employs the LOS method, which was proposed by Fossen(https://www.sciencedirect.com/science/article/pii/S1474667017378096). For communication part and information processing part, we use BPSK system and simple coding ways, including Parity Checking and Convolutional Codes.

## Contributers

This project is completed independently.
