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

- [`app/`](./app): Zephyr/NCS firmware application
- [`west.yml`](./west.yml): workspace manifest for NCS + Sidewalk add-on
- [`zephyr/module.yml`](./zephyr/module.yml): Zephyr module metadata for this repository

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

[`tools/ncs-env.sh`](tools/ncs-env.sh) sets up a working toolchain environment
(`PATH`, `LD_LIBRARY_PATH`, `ZEPHYR_BASE`, `ZEPHYR_TOOLCHAIN_VARIANT`,
`ZEPHYR_SDK_INSTALL_DIR`). Source it before running `west build` or any other
west command:

```sh
source ncs-sidewalk-demo-application/tools/ncs-env.sh
```

It picks the newest toolchain under `~/ncs/toolchains`. Set `NCS_TOOLCHAIN` to
choose a specific bundle, or `NCS_TOOLCHAIN_ROOT` if they live elsewhere. The
`LD_LIBRARY_PATH` entry is not optional: the bundle ships its own Python, and
west fails to start without it.

## Build the firmware

The primary demo build for the XIAO nRF54L15, DUT shell with the BLE NUS
companion shell, is:

```sh
west build -p always -b xiao_nrf54l15/nrf54l15/cpuapp ncs-sidewalk-demo-application/app \
  -d build/xiao-web-demo \
  -- -DOVERLAY_CONFIG='overlay-dut.conf;overlay-dut-nus.conf'
```

`overlay-dut.conf` enables the DUT command line, including the on-device
certificate commands. `overlay-dut-nus.conf` adds the separate BLE NUS shell
so a browser can talk to the shell over Web Bluetooth without taking over
the Sidewalk BLE identity. Other overlays under `app/`:

- `overlay-demo.conf`: sensor-monitoring demo build (buttons, LEDs, sensors)
- `overlay-hello.conf`, `overlay-hello-xiao-compare.conf`: minimal hello-world style builds
- `overlay-xiao-usb-console.overlay`: routes the console to USB CDC-ACM on the XIAO board
- `overlay-memfault.conf`: adds Memfault device health reporting, maintained separately. Check the overlay file itself for what it enables.

Combine overlays with a `;`-separated `-DOVERLAY_CONFIG` list as shown above.

## Provisioning status

The `prov status` shell command reports whether the device found a valid
Sidewalk manufacturing page at boot. It is available on both the UART shell
and the BLE NUS shell (`overlay-dut-nus.conf`). It prints a human-readable
summary and a machine-readable line:

```
EVT:{"t":"prov","provisioned":<true|false>,"smsn":"<64 hex chars or empty>","mfg_ver":<mfg store version>}
```

- `provisioned`: whether the manufacturing page was valid at the last boot.
- `smsn`: the Sidewalk manufacturing serial number as 64 uppercase hex characters, or an empty string when unprovisioned.
- `mfg_ver`: the raw manufacturing store version. `4294967295` (`0xFFFFFFFF`) means the mfg page is erased or was never written.

The same line is emitted automatically a few seconds after boot and again
shortly after a BLE NUS client connects, so a provisioning web app does not
have to send a command to learn the state.

An unprovisioned device keeps its UART shell and BLE NUS shell running; it
just never starts the Sidewalk stack. The manufacturing store version is
read once at boot and cached, so writing new credentials has no effect
until the device is rebooted. Reset the device after provisioning before
checking `prov status` again.

## Writing credentials over BLE NUS

There is no debug probe or SWD path in this project. A web app connects
over BLE NUS and writes credentials through the Sidewalk manufacturing
store API (`sid_pal_mfg_store_write()`), one named value at a time, using
the `prov` shell command group. This needs
`CONFIG_SIDEWALK_MFG_STORAGE_DIAGNOSTIC=y`, which `overlay-dut.conf` enables
through `CONFIG_SIDEWALK_ON_DEV_CERT=y`. If that config is missing, every
write command fails loudly instead of silently doing nothing.

Command sequence:

```
prov erase
prov set <value_id> <total_len> <frag_index> <base64>   (repeat per value)
prov finalize
prov reboot
```

`prov erase` clears the manufacturing store and any in-progress `prov set`
session. Always start a provisioning run with it.

`prov set <value_id> <total_len> <frag_index> <base64>` writes one value,
named by its numeric `sid_pal_mfg_store_value_t` id (see the table below).
`total_len` is the full decoded byte length of the value. `frag_index`
starts at 0 and must arrive in order; the firmware accumulates decoded
bytes in a small buffer and only calls into the manufacturing store once
the fragments received add up to `total_len`, then reads the value back
to confirm it landed correctly. Every value defined below is 64 bytes or
smaller, which base64-encodes to at most 88 characters, so in practice one
`prov set` call with `frag_index 0` carries the whole value; multi-fragment
sends are only needed if a client chooses to split a value on its own.
Sending a value id a second time overwrites the pending fragment session,
so retries are safe. Each completed value produces:

```
EVT:{"t":"provwr","id":<value_id>,"ok":<true|false>}
```

`prov finalize` checks that every required value id has been written and
(for non-secret ids) read back correctly, refusing with a named list of
what is missing if not. If everything is present, it writes only the
version/magic header through `sid_pal_mfg_store_write()`. It deliberately
does not write the manufacturing flags entry (`MFG_FLAGS_TYPE_ID`).
Leaving it absent makes `sid_pal_mfg_store_init()` read `-ENODATA` for it
on the next boot, which triggers `parse_mfg_raw_tlv()` (see the private
keys section below) to migrate the two raw private keys into PSA and write
the flags itself with the correct values. Claiming `keys_in_psa = 1`
ourselves at this point would be false (the keys are still plaintext TLV
entries in flash, not PSA-resident, until that migration runs) and would
skip the migration outright, leaving a device with no usable signing keys
and no sign that anything had gone wrong.

The version header is still written on its own, and still last:
`sid_pal_mfg_store_init()` only considers the device provisioned once it
can read the `SID0` magic and version at the very start of the partition,
and `parse_mfg_raw_tlv()` only starts reading TLV entries right after that
same header, so it has to exist for either check to do anything. If
finalize is interrupted before this write, the header still never
matched, so the next boot reports the device unprovisioned rather than
provisioned with missing or un-migrated data. This is the reverse of the
order used by the SDK's own `sid_on_dev_cert_verify_and_store()` path
(`cert store` on this shell), which writes the version right after
erasing and can leave a device that claims to be provisioned when the
write was in fact interrupted. Finalize reports:

```
EVT:{"t":"provdone","ok":<true|false>,"err":"<short reason or empty>"}
```

`prov reboot` does a clean reboot. Credentials only take effect on the
next boot, so always reboot after a successful finalize before trusting
`prov status`. That first boot after provisioning is not just "turn
Sidewalk on": it runs the SDK's own `parse_mfg_raw_tlv()` migration
(`sid_mfg_hex_v8.c`, the same path a `certificate_mfg.hex` flashed offline
goes through), which imports the two device private keys into PSA secure
storage and rewrites the manufacturing page without them. After that
first boot, the private keys no longer exist anywhere in flash in
plaintext, only inside PSA.

Required value ids for a complete provisioning (decimal id, name used in
error messages, expected byte length):

| id | name | bytes |
| -- | ---- | ----- |
| 4  | smsn | 32 |
| 38 | apid | 4 |
| 5  | app_pub_ed25519 | 32 |
| 6  | device_priv_ed25519 | 32 |
| 7  | device_pub_ed25519 | 32 |
| 8  | device_pub_ed25519_sig | 64 |
| 9  | device_priv_p256r1 | 32 |
| 10 | device_pub_p256r1 | 64 |
| 11 | device_pub_p256r1_sig | 64 |
| 12 | dak_pub_ed25519 | 32 |
| 13 | dak_pub_ed25519_sig | 64 |
| 14 | dak_ed25519_serial | 4 to 20 |
| 15 | dak_pub_p256r1 | 64 |
| 16 | dak_pub_p256r1_sig | 64 |
| 17 | dak_p256r1_serial | 4 to 20 |
| 18 | product_pub_ed25519 | 32 |
| 19 | product_pub_ed25519_sig | 64 |
| 20 | product_ed25519_serial | 4 to 20 |
| 21 | product_pub_p256r1 | 64 |
| 22 | product_pub_p256r1_sig | 64 |
| 23 | product_p256r1_serial | 4 to 20 |
| 24 | man_pub_ed25519 | 32 |
| 25 | man_pub_ed25519_sig | 64 |
| 26 | man_ed25519_serial | 4 to 20 |
| 27 | man_pub_p256r1 | 64 |
| 28 | man_pub_p256r1_sig | 64 |
| 29 | man_p256r1_serial | 4 to 20 |
| 30 | sw_pub_ed25519 | 32 |
| 31 | sw_pub_ed25519_sig | 64 |
| 32 | sw_ed25519_serial | 4 to 20 |
| 33 | sw_pub_p256r1 | 64 |
| 34 | sw_pub_p256r1_sig | 64 |
| 35 | sw_p256r1_serial | 4 to 20 |
| 36 | amzn_pub_ed25519 | 32 |
| 37 | amzn_pub_p256r1 | 64 |

The `*_serial` entries are certificate serial numbers. Sidewalk defines a
standard 4-byte encoding and an extended encoding for longer serials;
`prov set` accepts 4 to 20 bytes for these ids.

Ids 6 and 9 are the raw device private keys. In on-device certificate
generation these are born inside PSA and never touch flash, which is why
`sid_on_dev_cert_verify_and_store()` skips them under
`CONFIG_SIDEWALK_CRYPTO_PSA_KEY_STORAGE`. This flow is different: the
private keys come from AWS as part of the certificate, so they have to be
written to the mfg store as plain TLV values like everything else, and
the SDK's own first-boot migration (see above) moves them into PSA from
there. `prov set` still writes ids 6 and 9 through
`sid_pal_mfg_store_write()`, but skips the write-then-read-back check for
them (`sid_pal_mfg_store_read()` for these ids is intercepted by a PSA key
lookup that cannot succeed until after that migration has run, so a
read-back would misreport failure on a perfectly good write) and never
prints their byte length, only that they were stored.

## Web dashboard

The web dashboard now lives in the standalone repository:

- [`hlord2000/Sidewalk_Demo_Web_BLE`](https://github.com/hlord2000/Sidewalk_Demo_Web_BLE)

## Notes

- This repository does not vendor the Sidewalk SDK sources; it consumes `sdk-sidewalk` as a normal west project.
- The firmware code in [`app/`](./app) remains under the Sidewalk/Nordic licensing already present in the upstream add-on.
