# ESP32 UWB DW3000

The DW3000 library in this repository was developed by NConcepts.

This project uses MakerFabs DW3000 UWB ESP32 modules.

This code is written for our [work]((https://www.arxiv.org/abs/2505.09393)) "UMotion: Uncertainty-driven Human Motion Estimation from Inertial and Ultra-wideband Units".

## Description

Anchor-tag (AT): measuring the distance from one tag to multiple anchors
![Anchor tag](fig/AT.png)

Distance-matrix (DM): measuring the distance matrix among all nodes
![Distance matrix](fig/DM.png)

```bash
src
├── at_dstwr  # Anchor tag: double-sided two way ranging
│   ├── main.cpp  # setup() and loop()
│   ├── uwb.cpp  # ranging protocal
│   └── uwb.h  # constants, declaration
├── at_sstwr # Anchor tag: single-sided two way ranging
│   ├── main.cpp
│   ├── uwb.cpp
│   └── uwb.h
├── dm_dstwr  # Distance matrix: double-sided two way ranging
│   ├── main.cpp
│   ├── uwb.cpp
│   └── uwb.h
├── dm_sstwr  # Distance matrix: single-sided two way ranging
│   ├── main.cpp
│   ├── uwb.cpp
│   └── uwb.h
└── platformio.ini
```

## Getting started

1. Clone the repository onto your local system.
2. Connect the ESP32 UWB3000 and modify `upload_port` and `monitor_port` in `platformio.ini`.
    > U1 is the initiator
    - AT: `env:at_sstwr/dstwr_u<1-6>`. 
    - DM: `env:dm_sstwr/dstwr_u<1-6>`
3. Modify `NUM_NODES` and `INTERVAL` in `uwb.h`
4. Upload the code `pio run -e <env name>` to each device

## Citation

If you find this code useful in your research, please cite:

```bibtex
@inproceedings{liu2025umotion,
  title={UMotion: Uncertainty-driven Human Motion Estimation from Inertial and Ultra-wideband Units},
  author={Liu, Huakun and Ota, Hiroki and Wei, Xin and Hirao, Yutaro and Perusquia-Hernandez, Monica and Uchiyama, Hideaki and Kiyokawa, Kiyoshi},
  booktitle={Proceedings of the Computer Vision and Pattern Recognition Conference},
  pages={7085--7094},
  year={2025}
}
```

## TODO

- [ ] IMU (BNO086)-UWB (DW3000) sensing

## Misc

Note: If you encounter any issues or have questions, feel free to open an issue. You may also contact me via the email address: liu.huakun.li0@is.naist.jp.
