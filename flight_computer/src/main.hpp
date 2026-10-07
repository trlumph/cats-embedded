/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "target.hpp"

#include <cstdint>

/* Random pattern to tell system to jump to bootloader */
constexpr uint32_t BOOTLOADER_MAGIC_PATTERN = 0xAA88BB77;

/* RTC backup register DR1: start the next boot in testing mode (one shot, set by the Ground Station) */
constexpr uint32_t TESTING_BOOT_MAGIC_PATTERN = 0x7E57B007;
