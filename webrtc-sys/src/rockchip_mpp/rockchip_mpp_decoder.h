/*
 * Copyright 2026 LiveKit
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * Rockchip MPP (Media Process Platform) hardware H.264 decoder for
 * RV1126B-class SoCs. Pulls inbound H.264 NAL packets out of libwebrtc,
 * shoves them at MPP via decode_put_packet, retrieves NV12 frames via
 * decode_get_frame, and surfaces them as I420 webrtc::VideoFrames so the
 * existing rendering path (DRM/KMS plane via I420→NV12 software in
 * board_loopback) keeps working unchanged.
 *
 * Phase 6.2 status: full impl. RGA zero-copy + dmabuf-direct path → 6.1.5.
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

class RockchipMppH264DecoderImpl : public VideoDecoder {
public:
  explicit RockchipMppH264DecoderImpl(const SdpVideoFormat &format);
  RockchipMppH264DecoderImpl(const RockchipMppH264DecoderImpl &) = delete;
  RockchipMppH264DecoderImpl &
  operator=(const RockchipMppH264DecoderImpl &) = delete;
  ~RockchipMppH264DecoderImpl() override;

  bool Configure(const Settings &settings) override;
  int32_t Decode(const EncodedImage &input_image, bool missing_frames,
                 int64_t render_time_ms) override;
  int32_t RegisterDecodeCompleteCallback(
      DecodedImageCallback *callback) override;
  int32_t Release() override;
  DecoderInfo GetDecoderInfo() const override;

private:
  // Open MppCtx for AVC decode + configure split mode + IO timeouts.
  // Returns 0 on success, otherwise MPP_RET.
  int initMppContext();
  void teardownMppContext();
  // Drain MPP one frame; returns true if a frame was decoded + delivered
  // to decoded_complete_callback_, false otherwise (timeout / info-change /
  // error). Caller should loop while we may still have buffered frames.
  bool drainOneFrame(uint32_t rtp_timestamp);

  // Format from SDP (profile-level-id, packetization-mode, etc.).
  const SdpVideoFormat format_;

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
