/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#include "rockchip_mpp_decoder_factory.h"
#include "rockchip_mpp_decoder.h"

#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <sys/stat.h>

#include "absl/strings/match.h"
#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "rtc_base/logging.h"

// MppCodingType comes via rockchip_mpp_decoder.h's "rk_mpi.h" → "rk_type.h".

namespace webrtc {

namespace {

// SDP codec name → MPP coding type. Returned as optional so Create() can
// fall through to nullptr (libwebrtc then picks a software fallback) for
// formats we mistakenly advertise but can't build a coder for.
std::optional<MppCodingType> codecNameToMppCoding(const std::string &name) {
  if (absl::EqualsIgnoreCase(name, kH264CodecName)) {
    return MPP_VIDEO_CodingAVC;
  }
  if (absl::EqualsIgnoreCase(name, kH265CodecName)) {
    return MPP_VIDEO_CodingHEVC;
  }
  if (absl::EqualsIgnoreCase(name, kVp8CodecName)) {
    return MPP_VIDEO_CodingVP8;
  }
  if (absl::EqualsIgnoreCase(name, kVp9CodecName)) {
    return MPP_VIDEO_CodingVP9;
  }
  return std::nullopt;
}

} // namespace

RockchipMppVideoDecoderFactory::RockchipMppVideoDecoderFactory() {
  RTC_LOG(LS_INFO) << "[mpp-dec] RockchipMppVideoDecoderFactory ctor";
}

RockchipMppVideoDecoderFactory::~RockchipMppVideoDecoderFactory() = default;

bool RockchipMppVideoDecoderFactory::IsSupported() {
  const char *opt_in = std::getenv("BOARD_LOOPBACK_USE_MPP");
  if (!opt_in || std::string(opt_in) != "1") {
    return false;
  }
  struct stat st;
  if (::stat("/dev/mpp_service", &st) != 0) {
    RTC_LOG(LS_WARNING) << "[mpp-dec] BOARD_LOOPBACK_USE_MPP=1 but "
                           "/dev/mpp_service missing — decoder disabled";
    return false;
  }
  return true;
}

std::unique_ptr<VideoDecoder>
RockchipMppVideoDecoderFactory::Create(const Environment & /*env*/,
                                       const SdpVideoFormat &format) {
  std::string params;
  for (const auto &kv : format.parameters) {
    params += kv.first + "=" + kv.second + " ";
  }
  std::optional<MppCodingType> coding = codecNameToMppCoding(format.name);
  if (!coding) {
    // Shouldn't happen — libwebrtc only calls Create for formats we
    // advertised. Bail loud so a vendor-side change to GetSupportedFormats
    // can't silently produce a half-working decoder.
    std::fprintf(stderr, "[mpp-dec] Create(%s) unsupported name — refusing\n",
                 format.name.c_str());
    return nullptr;
  }
  std::fprintf(stderr, "[mpp-dec] Create(%s %s) → coding=%d\n",
               format.name.c_str(), params.c_str(),
               static_cast<int>(*coding));
  return std::make_unique<RockchipMppVideoDecoderImpl>(format, *coding);
}

std::vector<SdpVideoFormat>
RockchipMppVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats;

  // ---- H.264 ----
  // Full Constrained Baseline / Baseline / Main × packetization-mode 0/1
  // matrix that RV1126B's hardware decoder can handle. profile-level-id
  // matching is strict (IsSameCodec), so we list every flavour libwebrtc
  // peers commonly negotiate.
  static constexpr H264Profile kH264Profiles[] = {
      H264Profile::kProfileConstrainedBaseline,
      H264Profile::kProfileBaseline,
      H264Profile::kProfileMain,
  };
  for (auto profile : kH264Profiles) {
    for (const char *pkt_mode : {"1", "0"}) {
      formats.push_back(CreateH264Format(profile, H264Level::kLevel3_1,
                                         pkt_mode,
                                         /*add_scalability_modes=*/false));
    }
  }

  // ---- H.265 / VP8 / VP9 ----
  // Phase 6.4: advertise the rest of the codecs MPP can decode in hardware
  // so the SDP negotiation doesn't force the publisher into H.264. The
  // SdpVideoFormat::Hxxx() / VPx() statics give the canonical fmtp strings
  // the rest of libwebrtc uses internally; using them keeps the negotiation
  // matching predictable.
  formats.push_back(SdpVideoFormat::H265());
  formats.push_back(SdpVideoFormat::VP8());
  formats.push_back(SdpVideoFormat::VP9Profile0());

  return formats;
}

} // namespace webrtc
