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
#include "modules/video_coding/codecs/vp8/include/vp8.h"
#include "modules/video_coding/codecs/vp9/include/vp9.h"
#include "rtc_base/logging.h"

// MppCodingType comes via rockchip_mpp_decoder.h's "rk_mpi.h" → "rk_type.h".

namespace webrtc {

namespace {

// SDP codec name → MPP coding type, for the codecs RV1126B's MPP can
// actually drive in hardware (AVC + HEVC). VP8/VP9 are intentionally
// absent here — they go through the libvpx software path in Create().
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
RockchipMppVideoDecoderFactory::Create(const Environment &env,
                                       const SdpVideoFormat &format) {
  std::string params;
  for (const auto &kv : format.parameters) {
    params += kv.first + "=" + kv.second + " ";
  }

  // VP8 / VP9: peers (notably the LiveKit Android SDK) default to VP8
  // for publish, and we can't drop the receive side because LiveKit
  // doesn't transcode. Hand these off to libvpx's software decoder so
  // the channel still delivers video — at the cost of some CPU. The
  // hardware path stays for the codecs MPP can actually drive.
  if (absl::EqualsIgnoreCase(format.name, kVp8CodecName)) {
    std::fprintf(stderr, "[mpp-dec] Create(VP8 %s) → libvpx software\n",
                 params.c_str());
    return CreateVp8Decoder(env);
  }
  if (absl::EqualsIgnoreCase(format.name, kVp9CodecName)) {
    std::fprintf(stderr, "[mpp-dec] Create(VP9 %s) → libvpx software\n",
                 params.c_str());
    return VP9Decoder::Create();
  }

  std::optional<MppCodingType> coding = codecNameToMppCoding(format.name);
  if (!coding) {
    std::fprintf(stderr, "[mpp-dec] Create(%s) unsupported name — refusing\n",
                 format.name.c_str());
    return nullptr;
  }
  std::fprintf(stderr, "[mpp-dec] Create(%s %s) → MPP coding=%d\n",
               format.name.c_str(), params.c_str(),
               static_cast<int>(*coding));
  return std::make_unique<RockchipMppVideoDecoderImpl>(format, *coding);
}

std::vector<SdpVideoFormat>
RockchipMppVideoDecoderFactory::GetSupportedFormats() const {
  std::vector<SdpVideoFormat> formats;

  // ---- H.264 (MPP hardware) ----
  // Full Constrained Baseline / Baseline / Main × packetization-mode 0/1
  // matrix the hardware decoder handles. profile-level-id matching is
  // strict (IsSameCodec), so we list every flavour peers commonly
  // negotiate rather than relying on the FFI's secondary match loop.
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

  // ---- H.265 (MPP hardware) ----
  formats.push_back(SdpVideoFormat::H265());

  // ---- VP8 / VP9 (libvpx software fallback) ----
  // RV1126B's MPP can't decode VP8/VP9 ("unable to create dec vp8/vp9
  // for soc rv1126b unsupported" from libmpp), but advertising them
  // anyway lets a peer that only publishes VP8/VP9 (LiveKit Android
  // default) still deliver video — Create() routes these to libvpx.
  // CPU cost: libvpx VP8 720p30 ≈ +30% on this single-core part; if a
  // user wants the hardware fast path back, switch the publisher to
  // H.264/H.265.
  formats.push_back(SdpVideoFormat::VP8());
  for (const auto &fmt : SupportedVP9DecoderCodecs()) {
    formats.push_back(fmt);
  }

  return formats;
}

} // namespace webrtc
