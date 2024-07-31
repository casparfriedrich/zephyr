# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32F412RE" "--speed=auto" "--reset-after-load")
board_runner_args(dfu-util "--pid=0483:df11" "--alt=0" "--dfuse")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/dfu-util.board.cmake)
