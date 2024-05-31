board_runner_args(jlink "--device=STM32L071CZ" "--speed=4000" "--dev-id=777971011" "--reset")
 
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)