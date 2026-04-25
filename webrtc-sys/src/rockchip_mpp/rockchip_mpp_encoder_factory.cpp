/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#include "rockchip_mpp_encoder_factory.h"
#include "rockchip_mpp_encoder.h"

#include <cstdlib>
#include <sys/stat.h>

#include "absl/strings/match.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "rtc_base/logging.h"

namespace webrtc {

RockchipMppVideoEncoderFactory::RockchipMppVideoEncoderFactory() {
  RTC_LOG(LS_INFO) << "[mpp] RockchipMppVideoEncoderFactory ctor";
}

RockchipMppVideoEncoderFactory::~RockchipMppVideoEncoderFactory() = default;

bool RockchipMppVideoEncoderFactory::IsSupported() {
  // Phase 6.1 SKELETON: opt-in via env var until real MPP path is wired
  // in 6.1.4. Even when opted in, we additionally require /dev/mpp_service
  // so we don't claim support on hosts without the Rockchip kernel driver.
  const char *opt_in = std::getenv("BOARD_LOOPBACK_USE_MPP");
  if (!opt_in || std::string(opt_in) != "1") {
    return false;
  }
  struct stat st;
  if (::stat("/dev/mpp_service", &st) != 0) {
    RTC_LOG(LS_WARNING) << "[mpp] BOARD_LOOPBACK_USE_MPP=1 but "
                           "/dev/mpp_service missing — disabling factory";
    return false;
  }
  RTC_LOG(LS_INFO) << "[mpp] /dev/mpp_service present — factory enabled";
  return true;
}

std::unique_ptr<VideoEncoder>
RockchipMppVideoEncoderFactory::Create(const Environment & /*env*/,
                                       const SdpVideoFormat &format) {
  return std::make_unique<RockchipMppH264EncoderImpl>(format);
}

std::vector<SdpVideoFormat>
RockchipMppVideoEncoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats;
  // Constrained Baseline H.264 — most compatible for WebRTC
  // (cf. nvidia_encoder_factory.cpp profiles to mirror)
  formats.push_back(CreateH264Format(H264Profile::kProfileConstrainedBaseline,
                                     H264Level::kLevel3_1, "1",
                                     /*add_scalability_modes=*/false));
  formats.push_back(CreateH264Format(H264Profile::kProfileBaseline,
                                     H264Level::kLevel3_1, "1",
                                     /*add_scalability_modes=*/false));
  return formats;
}

std::vector<SdpVideoFormat>
RockchipMppVideoEncoderFactory::GetImplementations() const {
  return GetSupportedFormats();
}

} // namespace webrtc
