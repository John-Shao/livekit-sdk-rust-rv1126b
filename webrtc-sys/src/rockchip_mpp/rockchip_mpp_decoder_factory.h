/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 */

#ifndef WEBRTC_ROCKCHIP_MPP_DECODER_FACTORY_H_
#define WEBRTC_ROCKCHIP_MPP_DECODER_FACTORY_H_

#include <memory>
#include <vector>

#include "api/environment/environment.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_decoder.h"
#include "api/video_codecs/video_decoder_factory.h"

namespace webrtc {

class RockchipMppVideoDecoderFactory : public VideoDecoderFactory {
public:
  RockchipMppVideoDecoderFactory();
  ~RockchipMppVideoDecoderFactory() override;

  // Same opt-in gate as the encoder factory (see rockchip_mpp_encoder_factory):
  // env BOARD_LOOPBACK_USE_MPP=1 + /dev/mpp_service present. We deliberately
  // share the gate so a single env flag flips both sides — encode + decode
  // either both go hardware or both stay software.
  static bool IsSupported();

  std::unique_ptr<VideoDecoder> Create(const Environment &env,
                                       const SdpVideoFormat &format) override;

  std::vector<SdpVideoFormat> GetSupportedFormats() const override;
};

} // namespace webrtc

#endif // WEBRTC_ROCKCHIP_MPP_DECODER_FACTORY_H_
