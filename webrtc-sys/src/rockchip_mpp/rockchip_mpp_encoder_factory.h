/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef WEBRTC_ROCKCHIP_MPP_ENCODER_FACTORY_H_
#define WEBRTC_ROCKCHIP_MPP_ENCODER_FACTORY_H_

#include <memory>
#include <vector>

#include "api/environment/environment.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder.h"
#include "api/video_codecs/video_encoder_factory.h"

namespace webrtc {

class RockchipMppVideoEncoderFactory : public VideoEncoderFactory {
public:
  RockchipMppVideoEncoderFactory();
  ~RockchipMppVideoEncoderFactory() override;

  // Probe librockchip_mpp + /dev/mpp_service. Defaults to false during
  // Phase 6.1 SKELETON so OpenH264 keeps winning. Override with env var
  // BOARD_LOOPBACK_USE_MPP=1 to opt into the (currently broken) hw path
  // for development testing.
  static bool IsSupported();

  std::unique_ptr<VideoEncoder>
  Create(const Environment &env, const SdpVideoFormat &format) override;

  std::vector<SdpVideoFormat> GetSupportedFormats() const override;
  std::vector<SdpVideoFormat> GetImplementations() const override;
};

} // namespace webrtc

#endif // WEBRTC_ROCKCHIP_MPP_ENCODER_FACTORY_H_
