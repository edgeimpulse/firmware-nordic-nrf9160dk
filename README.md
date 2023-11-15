# Edge Impulse firmware for Nordic Semiconductor nRF9160DK

[Edge Impulse](https://www.edgeimpulse.com) enables developers to create the next generation of intelligent device solutions with embedded Machine Learning. This repository contains the Edge Impulse firmware for the Nordic Semiconductor nRF9160DK development board, in combination with the ST X_NUCLEO-IKS02A1 shield. This combination supports all Edge Impulse device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board with Edge Impulse? No need to build this firmware. See [these instructions](https://docs.edgeimpulse.com/docs/development-platforms/officially-supported-mcu-targets/nordic-semi-nrf9160-dk) for prebuilt images and instructions, or use the [data forwarder](https://docs.edgeimpulse.com/docs/tools/edge-impulse-cli/cli-data-forwarder) to capture data from any sensor.

## Building the device firmware (native)

1. Install the [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-2.4.0/page/nrf/getting_started/installing.html) in a *separate* folder from this repository (e.g. `~/repos/ncs`).

2. Clone this repository:

    ```bash
    $ git clone https://github.com/edgeimpulse/firmware-nordic-nrf9160dk
    ```

3. **ONLY ONCE** Build the board controller firmware:

    ```bash
    $ cd board-controller/
    $ west build -b nrf9160dk_nrf52840@1.0.0
    ```

4. Build the application:

    ```bash
    $ west build -b nrf9160dk_nrf9160_ns@1.0.0
    ```

## Building the device firmware (Docker)

1. Clone this repository:

    ```bash
    $ git clone https://github.com/edgeimpulse/firmware-nordic-nrf9160dk
    ```

2. Build the Docker container:

    ```bash
    $ docker build -t edge-impulse-nordic .
    ```

3. **ONLY ONCE** Build the board controller firmware:

    ```bash
    $ docker run --rm -v $PWD:/app -w /app/board-controller edge-impulse-nordic west build -b nrf9160dk_nrf52840@1.0.0
    ```

4. Build the application:

    ```bash
    $ docker run --rm -v $PWD:/app edge-impulse-nordic west build -b nrf9160dk_nrf9160_ns@1.0.0
    ```

## Flashing (JLink Mass Storage)

1. **ONLY ONCE** Flash the board controller firmware:
    1. Ensure that the `PROG/DEBUG` switch is in `nRF52` postion.

        ![nRF9160DK PROG/DEBUG switch location](./doc/nrf9160dk-prog-sw.jpg)

    2. Connect the board and power on.
    3. Copy `board-controller/build/zephyr/zephyr.bin` to the `JLINK` mass storage device.

1. Flashing the application:
    1. Ensure that the `PROG/DEBUG` switch is in `nRF91` postion.

        ![nRF9160DK PROG/DEBUG switch location](./doc/nrf9160dk-prog-sw.jpg)

    2. Connect the board and power on.
    3. Copy `build/zephyr/zephyr.bin` to the `JLINK` mass storage device.

## Flashing (command line)

1. **ONLY ONCE** Flash the board controller firmware:
    1. Ensure that the `PROG/DEBUG` switch is in `nRF52` postion.

        ![nRF9160DK PROG/DEBUG switch location](./doc/nrf9160dk-prog-sw.jpg)

    2. Connect the board and power on.
    3. Flash the board controller firmware:

        ```bash
        $ cd board-controller/
        $ west flash
        ```

1. Flashing the application:
    1. Ensure that the `PROG/DEBUG` switch is in `nRF91` postion.

        ![nRF9160DK PROG/DEBUG switch location](./doc/nrf9160dk-prog-sw.jpg)

    2. Connect the board and power on.
    3. Flash the application:

        ```bash
        $ west flash
        ```

## Updating your ML model

1. [Train a model in Edge Impulse](https://docs.edgeimpulse.com).
2. On the **Deployment** page in the Studio, export as a C++ library.
3. Remove the content of `ei-model` directory in this repository.
4. Extract the downloaded zip with the C++ library into `ei-model` directory.
5. Rebuild the application.

## Using Remote Ingestion

This firmware is equipped with the Remote Ingestion functionality. It allows you to connect your device to the internet using the LTE connection and start ingesting data remotely from the Edge Impulse Studio!

To build the firmware with the Remote Ingestion, follow the steps above but insted of command:

```
west build -b nrf9160dk_nrf9160_ns@1.0.0
```

Run:

```
west build -b nrf9160dk_nrf9160_ns@1.0.0 -- -DEXTRA_CONF_FILE=overlay-remote-ingestion.conf
```

And then:
1. Flash the board and connect for the first time to studio.
2. Power off the board and insert the SIM card.
    > **Note:** Make sure your SIM card is active. You can use the iBasis SIM card shipped with the nRF9160DK or any other SIM card that supports LTE-M and/or NB-IoT.
3. Power on the board and wait for the connection to be established (nRF9160DK will blink LEDs).
4. Go to your project in Studio and click on **Devices** tab, you should see your device with green mark.

## Troubleshooting

1. In case of any issues, the nRF9160DK with Edge Impulse firmware is exposing a serial console on the first UART port (connection parameters 115200bps 8N1). Open the serial port and reset the device to see the boot messages.
2. If you are using the Remote Ingestion functionality and have problems with connection, in the `overlay-remote-ingestion.conf` file change:

    ```
    CONFIG_REMOTE_INGESTION_LOG_LEVEL_WRN=y
    ```

    to:

    ```
    CONFIG_REMOTE_INGESTION_LOG_LEVEL_DBG=y
    ```

    After rebuilding the firmware and flashing, you should see more detailed logs in the serial console.
