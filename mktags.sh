#! /bin/sh

ctags -R \
    include \
    src \
    .pio/libdeps/m5stick-c \
    "${HOME}/.platformio/packages/framework-arduinoespressif32/tools/sdk/esp32"
