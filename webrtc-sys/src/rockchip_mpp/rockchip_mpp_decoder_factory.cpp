/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#include "rockchip_mpp_decoder_factory.h"
#include "rockchip_mpp_decoder.h"

#include <cstdio>
#include <cstdlib>
#include <sys/stat.h>

#include "media/base/media_constants.h"
#include "modules/video_coding/codecs/h264/include/h264.h"
#include "rtc_base/logging.h"

namespace webrtc {

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
  // Once-per-stream stderr trace — confirms we beat OpenH264 in the FFI's
  // factory selection. RTC_LOG isn't routed to stdout in this build, so
  // stderr is the only place ops can grep for hardware-codec activation.
  std::string params;
  for (const auto &kv : format.parameters) {
    params += kv.first + "=" + kv.second + " ";
  }
  std::fprintf(stderr, "[mpp-dec] Create(%s %s)\n", format.name.c_str(),
               params.c_str());
  return std::make_unique<RockchipMppH264DecoderImpl>(format);
}

std::vector<SdpVideoFormat>
RockchipMppVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats;
  // Advertise the full H.264 profile × packetization-mode matrix that
  // RV1126B's hardware decoder can handle (Baseline / Main / High up to
  // 4K@60 per Rockchip docs). Level 3.1 is plenty for 720p30; we use the
  // same level here so the SDP fmtp string parses consistently with the
  // encoder side. libwebrtc clients commonly negotiate {Constrained
  // Baseline, Baseline, Main} × {packetization-mode 0, 1}, so cover all
  // six explicitly — the FFI's secondary match loop in
  // video_decoder_factory.cpp can already absorb packetization-mode
  // mismatches, but matching profile-level-id is strict (IsSameCodec).
  static constexpr H264Profile kProfiles[] = {
      H264Profile::kProfileConstrainedBaseline,
      H264Profile::kProfileBaseline,
      H264Profile::kProfileMain,
  };
  for (auto profile : kProfiles) {
    for (const char *pkt_mode : {"1", "0"}) {
      formats.push_back(CreateH264Format(profile, H264Level::kLevel3_1,
                                         pkt_mode,
                                         /*add_scalability_modes=*/false));
    }
  }
  return formats;
}

} // namespace webrtc
