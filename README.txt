#
# https://michiel.vanderwulp.be/esp32-c3-supermini-blink-platformio.html
# https://michiel.vanderwulp.be/platformio-core-cli-installation-linux.html
#

alias get_pio='source ~/.platformio/penv/bin/activate'
get_pio
pio run
pio run -t upload

