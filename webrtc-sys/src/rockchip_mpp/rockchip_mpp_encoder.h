/*
 * Copyright 2026 LiveKit
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * Rockchip MPP (Media Process Platform) hardware H.264 encoder for
 * RV1126B-class SoCs. Wraps librockchip_mpp's MppCtx + MppApi over
 * webrtc::VideoEncoder so libwebrtc's encoder selection picks it
 * up via RockchipMppVideoEncoderFactory.
 *
 * Phase 6.1 status: SKELETON ONLY. Constructors / Init / Release wired
 * but Encode() is a stub returning ERROR. RockchipMppVideoEncoderFactory
 * IsSupported() defaults to false so OpenH264 software path keeps
 * winning until Phase 6.1.4 fills in the real MPP API calls.
 */

#ifndef WEBRTC_ROCKCHIP_MPP_ENCODER_IMPL_H_
#define WEBRTC_ROCKCHIP_MPP_ENCODER_IMPL_H_

#include <memory>
#include <mutex>
#include <vector>

#include "absl/container/inlined_vector.h"
#include "api/video/i420_buffer.h"
#include "api/video/video_codec_constants.h"
#include "api/video_codecs/scalability_mode.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder.h"
#include "common_video/h264/h264_bitstream_parser.h"

// MPP API headers — provided by SDK external/mpp/inc/
struct MppApi;
typedef void *MppCtx;
typedef void *MppFrame;
typedef void *MppPacket;
typedef void *MppBufferGroup;

namespace webrtc {

class RockchipMppH264EncoderImpl : public VideoEncoder {
public:
  explicit RockchipMppH264EncoderImpl(const SdpVideoFormat &format);
  ~RockchipMppH264EncoderImpl() override;

  // VideoEncoder overrides
  int InitEncode(const VideoCodec *codec_settings,
                 const Settings &settings) override;
  int Release() override;
  int RegisterEncodeCompleteCallback(EncodedImageCallback *callback) override;
  int Encode(const VideoFrame &frame,
             const std::vector<VideoFrameType> *frame_types) override;
  void SetRates(const RateControlParameters &parameters) override;
  EncoderInfo GetEncoderInfo() const override;

private:
  // Open MppCtx + configure for H.264 encode at given resolution/fps/bps.
  // Returns 0 on success, non-zero MPP_RET on failure.
  int initMppContext(int width, int height, int fps, int target_bps,
                     int gop_len);
  void teardownMppContext();

  // Format from SDP (profile-level-id, packetization-mode, etc.)
  const SdpVideoFormat format_;

  // Owned by factory client; not freed by us
  EncodedImageCallback *encoded_callback_ = nullptr;

  // MPP runtime state. nullptr until InitEncode succeeds.
  MppCtx mpp_ctx_ = nullptr;
  MppApi *mpp_api_ = nullptr;
  MppBufferGroup frame_buf_group_ = nullptr; // input NV12 frames
  MppBufferGroup pkt_buf_group_ = nullptr;   // output H.264 NAL packets

  // Current configuration
  int width_ = 0;
  int height_ = 0;
  int fps_ = 30;
  int target_bps_ = 1'500'000;
  int gop_len_ = 60;

  // H.264 bitstream parser for SVC info / SPS-PPS extraction
  H264BitstreamParser h264_parser_;

  // Initialized flag — also gates Encode() against unconfigured state
  bool initialized_ = false;

  std::mutex mutex_;
};

} // namespace webrtc

#endif // WEBRTC_ROCKCHIP_MPP_ENCODER_IMPL_H_
