/*
 * Copyright 2026 LiveKit
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * Rockchip MPP (Media Process Platform) hardware video decoder for
 * RV1126B-class SoCs. Codec-generic — the same class drives H.264 / H.265
 * / VP8 / VP9 (selected via constructor's MppCodingType param). Pulls
 * inbound encoded packets out of libwebrtc, shoves them at MPP via
 * decode_put_packet, retrieves NV12 frames via decode_get_frame, and
 * surfaces them as NV12Buffer webrtc::VideoFrames so the SDK FFI's
 * cvt_nv12 path can pass them to the consumer with a single memcpy.
 */

#ifndef WEBRTC_ROCKCHIP_MPP_DECODER_IMPL_H_
#define WEBRTC_ROCKCHIP_MPP_DECODER_IMPL_H_

#include <cstdint>
#include <memory>
#include <mutex>

#include "api/video_codecs/sdp_video_format.h"
#include "api/video_codecs/video_decoder.h"
#include "common_video/include/video_frame_buffer_pool.h"

extern "C" {
#include "mpp_buffer.h"
#include "mpp_frame.h"
#include "mpp_packet.h"
#include "rk_mpi.h"
}

namespace webrtc {

class RockchipMppVideoDecoderImpl : public VideoDecoder {
public:
  // `coding` selects the MPP decode pipeline (MPP_VIDEO_CodingAVC /
  // CodingHEVC / CodingVP8 / CodingVP9). Caller (factory) maps SDP codec
  // name → MppCodingType; from this point on all decode paths are uniform.
  RockchipMppVideoDecoderImpl(const SdpVideoFormat &format,
                              MppCodingType coding);
  RockchipMppVideoDecoderImpl(const RockchipMppVideoDecoderImpl &) = delete;
  RockchipMppVideoDecoderImpl &
  operator=(const RockchipMppVideoDecoderImpl &) = delete;
  ~RockchipMppVideoDecoderImpl() override;

  bool Configure(const Settings &settings) override;
  int32_t Decode(const EncodedImage &input_image, bool missing_frames,
                 int64_t render_time_ms) override;
  int32_t RegisterDecodeCompleteCallback(
      DecodedImageCallback *callback) override;
  int32_t Release() override;
  DecoderInfo GetDecoderInfo() const override;

private:
  // Open MppCtx for the configured codec + IO timeouts (split mode is set
  // only for Annex-B byte-stream codecs where one packet may carry
  // multiple NAL units, i.e. AVC/HEVC; VP8/VP9 frames don't need it).
  int initMppContext();
  void teardownMppContext();
  enum class DrainResult {
    kNothing,     // timeout / null / error — exit drain loop
    kInfoChange,  // size-change handshake done, retry immediately for the
                  // real frame queued behind it
    kFrame,       // delivered a real frame to decoded_complete_callback_;
                  // exit the drain loop unless caller knows more are pending
  };
  DrainResult drainOneFrame(uint32_t rtp_timestamp);

  // Format from SDP (profile-level-id, packetization-mode, etc.).
  const SdpVideoFormat format_;
  // Which MPP decoder pipeline to drive.
  const MppCodingType coding_;

  // MPP runtime state. nullptr until Configure() succeeds.
  MppCtx mpp_ctx_ = nullptr;
  MppApi *mpp_api_ = nullptr;
  MppBufferGroup frm_grp_ = nullptr; // external output frame group
  MppPacket packet_ = nullptr;       // reusable input packet handle

  // Output dimensions revealed by the first info-change frame.
  int width_ = 0;
  int height_ = 0;

  DecodedImageCallback *decoded_complete_callback_ = nullptr;
  webrtc::VideoFrameBufferPool buffer_pool_;
  bool initialized_ = false;
  std::mutex mutex_;
};

} // namespace webrtc

#endif // WEBRTC_ROCKCHIP_MPP_DECODER_IMPL_H_
