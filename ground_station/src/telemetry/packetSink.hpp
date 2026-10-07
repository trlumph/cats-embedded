/// Copyright (C) 2020, 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "telemetryData.hpp"

class ITelemetryPacketSink {
 public:
  ITelemetryPacketSink() = default;
  ITelemetryPacketSink(const ITelemetryPacketSink&) = delete;
  ITelemetryPacketSink& operator=(const ITelemetryPacketSink&) = delete;
  ITelemetryPacketSink(ITelemetryPacketSink&&) = delete;
  ITelemetryPacketSink& operator=(ITelemetryPacketSink&&) = delete;
  virtual ~ITelemetryPacketSink() = default;
  virtual void onTelemetryPacket(const packedRXMessage& packet, uint8_t source) = 0;
};

/// Receives every radio packet of a link while a Vega remote configuration session is open.
class IRemoteConfigSink {
 public:
  IRemoteConfigSink() = default;
  IRemoteConfigSink(const IRemoteConfigSink&) = delete;
  IRemoteConfigSink& operator=(const IRemoteConfigSink&) = delete;
  IRemoteConfigSink(IRemoteConfigSink&&) = delete;
  IRemoteConfigSink& operator=(IRemoteConfigSink&&) = delete;
  virtual ~IRemoteConfigSink() = default;
  /// payload holds 15 bytes; configFrame is true for remote configuration frames.
  virtual void onRadioPacket(uint8_t source, const uint8_t* payload, bool configFrame) = 0;
};
