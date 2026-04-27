/*
 * Copyright 2026 LiveKit
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * Rockchip MPP (Media Process Platform) hardware video encoder for
 * RV1126B-class SoCs. Codec-generic — same class drives H.264 (AVC)
 * and H.265 (HEVC) encode pipelines on this SoC; coding is selected
 * by the factory via the constructor's MppCodingType. Wraps
 * librockchip_mpp's MppCtx + MppApi over webrtc::VideoEncoder so
 * libwebrtc's encoder selection picks it up via the factory.
 *
 * Phase 6.1.4: real H.264 encode path landed (I420→NV12 + put_frame /
 * get_packet). Phase 7 generalizes the same loop to HEVC by branching
 * codec-specific MppEncCfg keys + IDR/parameter-set NAL detection.
 */

#ifndef WEBRTC_ROCKCHIP_MPP_ENCODER_IMPL_H_
#define WEBRTC_ROCKCHIP_MPP_ENCODER_IMPL_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "api/video/i420_buffer.h"
#include "api/video/video_codec_constants.h"
#include "api/video_codecs/scalability_mode.h"
#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_encoder.h"
#include "common_video/h264/h264_bitstream_parser.h"
#include "common_video/h265/h265_bitstream_parser.h"

// MPP API headers — provided by SDK external/mpp/inc/, made visible by
// build.rs adding ROCKCHIP_MPP_INCLUDE to the include path. Pull in
// rk_mpi.h here so we can store an MppApi* member; the rest are also
// dragged in directly because we deref into them in the .cpp anyway and
// keeping the header self-consistent avoids forward-decl/typedef drift.
extern "C" {
#include "mpp_buffer.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "rk_mpi.h"
#include "rk_venc_cfg.h"
}

namespace webrtc {

class RockchipMppVideoEncoderImpl : public VideoEncoder {
public:
  // `coding` selects the MPP encode pipeline (MPP_VIDEO_CodingAVC or
  // CodingHEVC). Caller (factory) maps SDP codec name → MppCodingType
  // via the same name table the decoder factory uses.
  RockchipMppVideoEncoderImpl(const SdpVideoFormat &format,
                              MppCodingType coding);
  ~RockchipMppVideoEncoderImpl() override;

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
  // Apply current target_bps_ / fps_ to the live MPP cfg via MPP_ENC_SET_CFG.
  // Returns 0 on success, otherwise MPP_RET.
  int applyRateControl();

  // Format from SDP (profile-level-id, packetization-mode, etc.)
  const SdpVideoFormat format_;
  // Which MPP encoder pipeline to drive.
  const MppCodingType coding_;

  // Owned by factory client; not freed by us
  EncodedImageCallback *encoded_callback_ = nullptr;

  // MPP runtime state. nullptr until InitEncode succeeds.
  MppCtx mpp_ctx_ = nullptr;
  MppApi *mpp_api_ = nullptr;
  MppBufferGroup buf_grp_ = nullptr;
  MppBuffer frm_buf_ = nullptr; // input NV12 frame
  MppBuffer pkt_buf_ = nullptr; // output H.264 NAL packet
  MppEncCfg cfg_ = nullptr;

  // Current configuration
  int width_ = 0;
  int height_ = 0;
  int hor_stride_ = 0; // 16-aligned width (bytes per Y row)
  int ver_stride_ = 0; // 16-aligned height (Y rows total)
  int fps_ = 30;
  int target_bps_ = 1'500'000;
  int gop_len_ = 60;

  // Parameter sets cached after init via MPP_ENC_GET_HDR_SYNC; libwebrtc
  // expects them prepended to every IDR access unit so a late-joining
  // receiver can decode without waiting for a fresh SPS/PPS (or VPS+SPS+PPS
  // for H.265).
  std::vector<uint8_t> hdr_pps_sps_;

  // Bitstream parsers for QP extraction (best-effort — used only for
  // stats/pacer feedback). One of these is exercised per Encode() call
  // depending on coding_; the unused one stays a no-op.
  H264BitstreamParser h264_parser_;
  H265BitstreamParser h265_parser_;

  // Initialized flag — also gates Encode() against unconfigured state
  bool initialized_ = false;

  std::mutex mutex_;
};

} // namespace webrtc

#endif // WEBRTC_ROCKCHIP_MPP_ENCODER_IMPL_H_
