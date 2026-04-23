# ncs-sidewalk-demo-application

# For info on web service: https://github.com/hlord2000/Sidewalk_Demo_Web_BLE

Required items:
 - nRF54L15 XIAO -  https://www.seeedstudio.com/XIAO-nRF54L15-p-6493.html
 - Wio-SX1262 for XIAO - https://www.seeedstudio.com/Wio-SX1262-for-XIAO-p-6379.html
 - Battery (optional)

Standalone nRF Connect SDK application repository for the Sidewalk demo setup.

The repository is intentionally shaped like
[`ncs-example-application`](https://github.com/nrfconnect/ncs-example-application):

- top-level `west.yml` manifest
- top-level Zephyr module metadata
- application sources under [`app/`](./app)
- companion tooling under [`tools/`](./tools)

This repository packages:

- the extracted Sidewalk end-device demo firmware
- the XIAO nRF54L15 web-demo BLE shell and button-trigger additions

## Repository layout

- [`app/`](/opt/ncs/sdks/ncs-sdk-sidewalk/ncs-sidewalk-demo-application/app): Zephyr/NCS firmware application
- [`west.yml`](/opt/ncs/sdks/ncs-sdk-sidewalk/ncs-sidewalk-demo-application/west.yml): workspace manifest for NCS + Sidewalk add-on
- [`zephyr/module.yml`](/opt/ncs/sdks/ncs-sdk-sidewalk/ncs-sidewalk-demo-application/zephyr/module.yml): Zephyr module metadata for this repository

## Workspace initialization

Initialize a fresh workspace using this repository as the manifest:

```sh
west init -m <your-repo-url> --mr main ncs-sidewalk-demo
cd ncs-sidewalk-demo
west update
pip install -r sidewalk/requirements.txt
```

This manifest pulls:

- `nrfconnect/sdk-nrf` at `v3.0.0`
- `nrfconnect/sdk-sidewalk` at `v1.1.0-add-on`

## Build the firmware

The primary demo build for the XIAO nRF54L15 is:

```sh
west build -p always -b xiao_nrf54l15/nrf54l15/cpuapp app \
  -d build/xiao-web-demo \
  -- \
  -DFILE_SUFFIX=release \
  -DOVERLAY_CONFIG='overlay-min-size.conf;overlay-prop-radio.conf;overlay-web-demo.conf' \
  -DDTC_OVERLAY_FILE='boards/xiao_nrf54l15_nrf54l15_cpuapp.overlay;overlay-web-demo.overlay'
```

That build includes:

- Sidewalk BLE + FSK + LoRa
- separate BLE NUS shell for Web Bluetooth
- proprietary radio handoff support
- button-triggered Sidewalk send on `P1.04`
- LED transport indication on `P2.00`

## Web dashboard

The web dashboard now lives in the standalone repository:

- [`hlord2000/Sidewalk_Demo_Web_BLE`](https://github.com/hlord2000/Sidewalk_Demo_Web_BLE)

## Notes

- This repository does not vendor the Sidewalk SDK sources; it consumes `sdk-sidewalk` as a normal west project.
- The firmware code in [`app/`](/opt/ncs/sdks/ncs-sdk-sidewalk/ncs-sidewalk-demo-application/app) remains under the Sidewalk/Nordic licensing already present in the upstream add-on.
