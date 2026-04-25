/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#include "rockchip_mpp_decoder_factory.h"
#include "rockchip_mpp_decoder.h"

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
  return std::make_unique<RockchipMppH264DecoderImpl>(format);
}

std::vector<SdpVideoFormat>
RockchipMppVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats;
  // Mirror the encoder side — we only advertise H.264 baseline / CB at
  // Level 3.1 since that's what RV1126B's H.264 decoder reliably handles
  // in our smoke tests. WebRTC will auto-fall-back to software for higher
  // profiles (Main/High) via the parent VideoDecoderFactory.
  formats.push_back(CreateH264Format(H264Profile::kProfileConstrainedBaseline,
                                     H264Level::kLevel3_1, "1",
                                     /*add_scalability_modes=*/false));
  formats.push_back(CreateH264Format(H264Profile::kProfileBaseline,
                                     H264Level::kLevel3_1, "1",
                                     /*add_scalability_modes=*/false));
  return formats;
}

} // namespace webrtc
