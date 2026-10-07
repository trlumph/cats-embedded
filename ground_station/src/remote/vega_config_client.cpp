/// Copyright (C) 2026 Control and Telemetry Systems GmbH
///
/// SPDX-License-Identifier: GPL-3.0-or-later

#include "remote/vega_config_client.hpp"

#include <cstdio>
#include <cstring>

#include "telemetry/crc.hpp"

namespace VegaConfig {

using remote_config::DataFrame;
using remote_config::DownlinkType;
using remote_config::kDataChunkSize;
using remote_config::kFrameSize;
using remote_config::kLineChunkSize;
using remote_config::kMaxLineLength;
using remote_config::kMaxTextLength;
using remote_config::Op;
using remote_config::Result;
using remote_config::StatusFrame;

void Client::begin(uint32_t sessionPasscode, uint8_t seqSeed) {
  passcode = sessionPasscode;
  seq = seqSeed == 0 ? 1 : seqSeed;
  currentTask = Task::kNone;
  currentState = State::kIdle;
  step = Step::kIdle;
  statusSeen = false;
  textReady = false;
  errorText[0] = '\0';
  request = {};
  request.passcode = passcode;
}

uint8_t Client::nextSeq() {
  seq = static_cast<uint8_t>(seq + 1U);
  if (seq == 0) {
    seq = 1;
  }
  return seq;
}

void Client::fail(const char* message) {
  snprintf(errorText, sizeof(errorText), "%s", message);
  currentState = State::kFailed;
  step = Step::kIdle;
  request.op = Op::kNop;
}

void Client::finish() {
  currentState = State::kDone;
  step = Step::kIdle;
  request.op = Op::kNop;
}

void Client::stop() {
  currentTask = Task::kNone;
  currentState = State::kIdle;
  step = Step::kIdle;
  request.op = Op::kNop;
}

void Client::startOp(Op op, uint32_t now) {
  request = {};
  request.passcode = passcode;
  request.op = op;
  request.seq = nextSeq();
  step = Step::kWaitAck;
  stepStarted = now;
}

void Client::startConnect(uint32_t now) {
  currentTask = Task::kConnect;
  currentState = State::kBusy;
  errorText[0] = '\0';
  statusSeen = false;
  request = {};
  request.passcode = passcode;
  request.op = Op::kNop;
  step = Step::kWaitStatus;
  stepStarted = now;
}

void Client::resetCollection() {
  memset(received, 0, sizeof(received));
  memset(textBuffer, 0, sizeof(textBuffer));
  lengthKnown = false;
  textReady = false;
  expectedLength = 0;
  expectedCrc = 0;
}

void Client::startRead(uint32_t now) {
  currentTask = Task::kRead;
  currentState = State::kBusy;
  errorText[0] = '\0';
  resetCollection();
  startOp(Op::kRead, now);
}

void Client::startWrite(const std::vector<std::string>& writeLines, uint32_t now) {
  currentTask = Task::kWrite;
  currentState = State::kBusy;
  errorText[0] = '\0';
  lines = writeLines;
  lineIndex = 0;
  failedLineIndex = -1;
  committing = false;
  for (size_t index = 0; index < lines.size(); ++index) {
    if (lines[index].empty() || lines[index].size() > kMaxLineLength) {
      failedLineIndex = static_cast<int>(index);
      fail("Setting line too long");
      return;
    }
  }
  if (!statusSeen) {
    fail("No status from Vega");
    return;
  }
  startOp(Op::kBegin, now);
}

void Client::startReboot(remote_config::RebootMode mode, uint32_t now) {
  currentTask = Task::kReboot;
  currentState = State::kBusy;
  errorText[0] = '\0';
  rebootMode = mode;
  if (!statusSeen) {
    fail("No status from Vega");
    return;
  }
  startOp(Op::kReboot, now);
  request.args[0] = lastStatus.token;
  request.args[1] = static_cast<uint8_t>(mode);
  memcpy(&request.args[2], remote_config::kRebootMagic, sizeof(remote_config::kRebootMagic));
}

void Client::startLine(uint32_t now) {
  chunkOffset = 0;
  chunkRepeated = false;
  step = Step::kLineChunks;
  stepStarted = now;
}

void Client::handleAck(Result result, uint32_t now) {
  switch (currentTask) {
    case Task::kRead:
      if (result != Result::kOk) {
        fail(remote_config::ResultName(result));
        return;
      }
      step = Step::kCollect;
      stepStarted = now;
      return;
    case Task::kReboot:
      if (result == Result::kBadToken && restartedSinceRequest()) {
        // The acknowledgement was lost, the Vega already restarted and now rejects the repeated request.
        finish();
        return;
      }
      if (result != Result::kOk) {
        fail(remote_config::ResultName(result));
        return;
      }
      finish();
      return;
    case Task::kWrite:
      if (request.op == Op::kBegin) {
        if (result != Result::kOk) {
          fail(remote_config::ResultName(result));
          return;
        }
        if (lines.empty()) {
          committing = true;
        } else {
          lineAttempts = 0;
          startLine(now);
          return;
        }
      } else if (request.op == Op::kLineApply) {
        if (result == Result::kBadCrc && ++lineAttempts < kLineRetries) {
          startLine(now);
          return;
        }
        if (result != Result::kOk) {
          failedLineIndex = static_cast<int>(lineIndex);
          char message[64]{};
          const std::string& line = lines[lineIndex];
          const size_t keyLength = line.find('=');
          snprintf(message, sizeof(message), "%s: %.*s", remote_config::ResultName(result),
                   static_cast<int>(keyLength == std::string::npos ? 20 : keyLength), line.c_str());
          fail(message);
          return;
        }
        ++lineIndex;
        if (lineIndex < lines.size()) {
          lineAttempts = 0;
          startLine(now);
          return;
        }
        committing = true;
      } else if (request.op == Op::kCommit) {
        if (result == Result::kBadToken && restartedSinceRequest()) {
          // Lost acknowledgement; the session verifies the values after the restart.
          finish();
          return;
        }
        if (result != Result::kOk) {
          fail(remote_config::ResultName(result));
          return;
        }
        finish();
        return;
      }
      if (committing) {
        startOp(Op::kCommit, now);
        request.args[0] = lastStatus.token;
        memcpy(&request.args[1], remote_config::kCommitMagic, sizeof(remote_config::kCommitMagic));
      }
      return;
    default:
      return;
  }
}

bool Client::restartedSinceRequest() const { return lastStatus.token != request.args[0]; }

void Client::handleStatus(const StatusFrame& status, uint32_t now) {
  if (status.version != remote_config::kProtocolVersion) {
    if (currentState == State::kBusy) {
      fail("Vega uses another protocol version");
    }
    return;
  }
  lastStatus = status;
  statusSeen = true;
  lastStatusAt = now;
  if (currentState != State::kBusy) {
    return;
  }
  if (step == Step::kWaitStatus) {
    finish();
    return;
  }
  const bool readAck = currentTask == Task::kRead && status.seq == request.seq && status.result == Result::kOk;
  if (step == Step::kWaitAck && status.seq == request.seq && status.result != Result::kNone) {
    handleAck(status.result, now);
    if (!readAck) {
      return;
    }
  }
  if (step != Step::kCollect || !readAck ||
      (lengthKnown && expectedCrc == status.textCrc && expectedLength == status.textLength)) {
    return;
  }
  if (status.textLength == 0 || status.textLength > kMaxTextLength) {
    fail("Invalid configuration size");
    return;
  }
  if (lengthKnown) {
    // The Vega rebuilt its text; start over.
    memset(received, 0, sizeof(received));
  }
  expectedLength = status.textLength;
  expectedCrc = status.textCrc;
  lengthKnown = true;
}

void Client::handleData(const DataFrame& data) {
  if (currentState != State::kBusy || currentTask != Task::kRead) {
    return;
  }
  if (data.offset % kDataChunkSize != 0 || data.offset >= kMaxTextLength) {
    return;
  }
  const size_t chunk = data.offset / kDataChunkSize;
  const size_t count = kMaxTextLength - data.offset < kDataChunkSize ? kMaxTextLength - data.offset : kDataChunkSize;
  memcpy(&textBuffer[data.offset], data.data, count);
  received[chunk] = true;
}

void Client::checkReadComplete() {
  if (currentState != State::kBusy || currentTask != Task::kRead || !lengthKnown || expectedLength == 0) {
    return;
  }
  const size_t chunks = (expectedLength + kDataChunkSize - 1) / kDataChunkSize;
  for (size_t chunk = 0; chunk < chunks; ++chunk) {
    if (!received[chunk]) {
      return;
    }
  }
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  if (crc32(reinterpret_cast<const uint8_t*>(textBuffer), expectedLength) != expectedCrc) {
    memset(received, 0, sizeof(received));
    return;
  }
  textBuffer[expectedLength] = '\0';
  textReady = true;
  finish();
}

void Client::onDownlink(const uint8_t* frame, uint32_t now) {
  const DownlinkType type = remote_config::DownlinkTypeOf(frame, kFrameSize);
  if (type == DownlinkType::kStatus) {
    StatusFrame status{};
    remote_config::DecodeStatus(frame, &status);
    handleStatus(status, now);
  } else if (type == DownlinkType::kData) {
    DataFrame data{};
    remote_config::DecodeData(frame, &data);
    handleData(data);
  }
  checkReadComplete();
}

void Client::buildFrame(uint8_t out[kFrameSize]) {
  if (step == Step::kLineChunks) {
    remote_config::UplinkFrame chunk{};
    chunk.passcode = passcode;
    chunk.op = Op::kLineData;
    const std::string& line = lines[lineIndex];
    const size_t remaining = line.size() - chunkOffset;
    const size_t length = remaining < kLineChunkSize ? remaining : kLineChunkSize;
    chunk.args[0] = static_cast<uint8_t>(chunkOffset);
    chunk.args[1] = static_cast<uint8_t>(length);
    memcpy(&chunk.args[2], line.data() + chunkOffset, length);
    remote_config::EncodeUplink(chunk, out);
    return;
  }
  remote_config::EncodeUplink(request, out);
}

bool Client::onPacket(uint8_t out[kFrameSize], uint32_t now) {
  if (step == Step::kLineChunks) {
    buildFrame(out);
    if (lineAttempts > 0 && !chunkRepeated) {
      // Retry: every chunk goes out twice.
      chunkRepeated = true;
      return true;
    }
    chunkRepeated = false;
    const std::string& line = lines[lineIndex];
    const size_t remaining = line.size() - chunkOffset;
    chunkOffset += remaining < kLineChunkSize ? remaining : kLineChunkSize;
    if (chunkOffset >= line.size()) {
      // All chunks are out; the next answer carries the apply request.
      startOp(Op::kLineApply, now);
      request.args[0] = lastStatus.token;
      request.args[1] = static_cast<uint8_t>(line.size());
      remote_config::PutU32(&request.args[2],
                            // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
                            crc32(reinterpret_cast<const uint8_t*>(line.data()), line.size()));
    }
    return true;
  }
  buildFrame(out);
  return true;
}

void Client::tick(uint32_t now) {
  if (currentState != State::kBusy) {
    return;
  }
  const uint32_t elapsed = now - stepStarted;
  switch (step) {
    case Step::kWaitStatus:
      if (elapsed > kConnectTimeoutMs) {
        fail("No answer from Vega");
      }
      break;
    case Step::kWaitAck:
    case Step::kLineChunks:
      if (elapsed > kAckTimeoutMs) {
        fail("Vega stopped answering");
      }
      break;
    case Step::kCollect:
      if (elapsed > kReadTimeoutMs) {
        fail("Reading timed out");
      }
      break;
    default:
      break;
  }
}

uint8_t Client::percent() const {
  if (currentState == State::kDone) {
    return 100;
  }
  if (currentTask == Task::kRead && lengthKnown && expectedLength > 0) {
    const size_t chunks = (expectedLength + kDataChunkSize - 1) / kDataChunkSize;
    if (chunks == 0) {
      return 0;
    }
    size_t done = 0;
    for (size_t chunk = 0; chunk < chunks; ++chunk) {
      done += received[chunk] ? 1U : 0U;
    }
    return static_cast<uint8_t>(done * 100U / chunks);
  }
  if (currentTask == Task::kWrite && !lines.empty()) {
    return static_cast<uint8_t>(lineIndex * 100U / (lines.size() + 1U));
  }
  return 0;
}

}  // namespace VegaConfig
