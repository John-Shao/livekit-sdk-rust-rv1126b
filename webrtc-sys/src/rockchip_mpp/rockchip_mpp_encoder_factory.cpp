/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#include "rockchip_mpp_encoder_factory.h"
#include "rockchip_mpp_encoder.h"

#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <sys/stat.h>

#include "absl/strings/match.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "rtc_base/logging.h"

// MppCodingType comes via rockchip_mpp_encoder.h's "rk_mpi.h" → "rk_type.h".

namespace webrtc {

namespace {

// SDP codec name → MPP coding type, mirrored from the decoder factory.
// RV1126B's MPP encoder front-end supports AVC + HEVC; VP8/VP9 encode
// is not implemented on this SoC so they're absent here.
std::optional<MppCodingType> codecNameToMppCoding(const std::string &name) {
  if (absl::EqualsIgnoreCase(name, kH264CodecName)) {
    return MPP_VIDEO_CodingAVC;
  }
  if (absl::EqualsIgnoreCase(name, kH265CodecName)) {
    return MPP_VIDEO_CodingHEVC;
  }
  return std::nullopt;
}

} // namespace

RockchipMppVideoEncoderFactory::RockchipMppVideoEncoderFactory() {
  RTC_LOG(LS_INFO) << "[mpp] RockchipMppVideoEncoderFactory ctor";
}

RockchipMppVideoEncoderFactory::~RockchipMppVideoEncoderFactory() = default;

bool RockchipMppVideoEncoderFactory::IsSupported() {
  // Phase 6.1 SKELETON gate retained: opt-in via env so a host without
  // the Rockchip kernel driver doesn't claim hardware support and
  // silently break encode chain selection.
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
  std::string params;
  for (const auto &kv : format.parameters) {
    params += kv.first + "=" + kv.second + " ";
  }
  std::optional<MppCodingType> coding = codecNameToMppCoding(format.name);
  if (!coding) {
    // libwebrtc only calls Create for formats we advertised — bail loud
    // so a future GetSupportedFormats expansion that forgets to update
    // codecNameToMppCoding can't half-instantiate the encoder.
    std::fprintf(stderr, "[mpp-enc] Create(%s) unsupported name — refusing\n",
                 format.name.c_str());
    return nullptr;
  }
  std::fprintf(stderr, "[mpp-enc] Create(%s %s) → coding=%d\n",
               format.name.c_str(), params.c_str(),
               static_cast<int>(*coding));
  return std::make_unique<RockchipMppVideoEncoderImpl>(format, *coding);
}

std::vector<SdpVideoFormat>
RockchipMppVideoEncoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats;

  // ---- H.264 ----
  // Constrained Baseline + Baseline at Level 3.1 (covers 720p30). Order
  // matters: WebRTC's offer SDP keeps factory order, and most peers fall
  // through the list left-to-right, so put the most compatible first.
  formats.push_back(CreateH264Format(H264Profile::kProfileConstrainedBaseline,
                                     H264Level::kLevel3_1, "1",
                                     /*add_scalability_modes=*/false));
  formats.push_back(CreateH264Format(H264Profile::kProfileBaseline,
                                     H264Level::kLevel3_1, "1",
                                     /*add_scalability_modes=*/false));

  // ---- H.265 ----
  // Phase 7: encoder side is now multi-codec. We advertise the canonical
  // H.265 SDP entry; cfg keys (Main / Tier 0 / Level 3.1) are set inside
  // the encoder to match.
  formats.push_back(SdpVideoFormat::H265());

  return formats;
}

std::vector<SdpVideoFormat>
RockchipMppVideoEncoderFactory::GetImplementations() const {
  return GetSupportedFormats();
}

} // namespace webrtc
