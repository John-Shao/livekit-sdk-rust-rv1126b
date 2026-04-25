/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 *
 * Phase 6.2 — Rockchip MPP H.264 decode pipeline:
 *   Configure  : mpp_create + mpp_init(MPP_CTX_DEC, AVC) + parser_split_mode
 *                + sane IO timeouts (input block, output 100ms)
 *   Decode     : wrap input EncodedImage bytes in our reusable MppPacket,
 *                decode_put_packet (retry briefly on EAGAIN), then
 *                decode_get_frame loop:
 *                  - info-change frame  → mpp_buffer_group_get_internal +
 *                    MPP_DEC_SET_EXT_BUF_GROUP + MPP_DEC_SET_INFO_CHANGE_READY
 *                  - real NV12 frame    → libyuv NV12→I420 → VideoFrame →
 *                    decoded_complete_callback_->Decoded()
 *
 * RGA zero-copy + dmabuf-direct rendering path → Phase 6.1.5.
 */

#define MODULE_TAG "livekit_mpp_dec"

#include "rockchip_mpp_decoder.h"

#include <chrono>
#include <cstring>
#include <thread>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/logging.h"
#include "third_party/libyuv/include/libyuv/convert.h"

extern "C" {
#include "mpp_err.h"
#include "rk_mpi_cmd.h"
}

namespace webrtc {

RockchipMppH264DecoderImpl::RockchipMppH264DecoderImpl(
    const SdpVideoFormat &format)
    : format_(format), buffer_pool_(/*zero_initialize=*/false, 16) {
  RTC_LOG(LS_INFO) << "[mpp-dec] RockchipMppH264DecoderImpl ctor";
}

RockchipMppH264DecoderImpl::~RockchipMppH264DecoderImpl() { Release(); }

bool RockchipMppH264DecoderImpl::Configure(const Settings & /*settings*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_)
    return true;
  int rc = initMppContext();
  if (rc != 0) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] initMppContext failed: " << rc;
    teardownMppContext();
    return false;
  }
  initialized_ = true;
  return true;
}

int32_t RockchipMppH264DecoderImpl::Release() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    teardownMppContext();
    initialized_ = false;
  }
  decoded_complete_callback_ = nullptr;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipMppH264DecoderImpl::RegisterDecodeCompleteCallback(
    DecodedImageCallback *callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  decoded_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipMppH264DecoderImpl::Decode(const EncodedImage &input_image,
                                           bool /*missing_frames*/,
                                           int64_t /*render_time_ms*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !mpp_ctx_ || !mpp_api_) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!decoded_complete_callback_) {
    return WEBRTC_VIDEO_CODEC_OK; // nobody listening
  }
  if (!input_image.data() || input_image.size() == 0) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }

  // 1. Wrap the inbound bitstream in our reusable MppPacket. The buffer
  //    is owned by the EncodedImage; MPP copies what it needs into its
  //    internal stream buffer before decode_put_packet returns, so the
  //    pointer doesn't have to stay live past this call.
  mpp_packet_set_data(packet_, const_cast<uint8_t *>(input_image.data()));
  mpp_packet_set_size(packet_, input_image.size());
  mpp_packet_set_pos(packet_, const_cast<uint8_t *>(input_image.data()));
  mpp_packet_set_length(packet_, input_image.size());

  // 2. Push the packet, retrying briefly if the parser is full (rare —
  //    input timeout was set to MPP_POLL_BLOCK at init).
  for (int tries = 0; tries < 50; ++tries) {
    MPP_RET ret = mpp_api_->decode_put_packet(mpp_ctx_, packet_);
    if (ret == MPP_OK) {
      break;
    }
    if (tries == 49) {
      RTC_LOG(LS_ERROR) << "[mpp-dec] decode_put_packet stuck: " << ret;
      return WEBRTC_VIDEO_CODEC_ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // 3. Drain whatever frames are ready. drainOneFrame returns true while
  //    something useful happens (a delivered frame, or absorbed
  //    info-change / discard). If everything's quiet we exit and let the
  //    caller's next packet trigger more output.
  for (int i = 0; i < 8; ++i) {
    if (!drainOneFrame(input_image.RtpTimestamp()))
      break;
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

VideoDecoder::DecoderInfo RockchipMppH264DecoderImpl::GetDecoderInfo() const {
  DecoderInfo info;
  info.implementation_name = "RockchipMpp_H264";
  info.is_hardware_accelerated = true;
  return info;
}

// ---- private helpers ---------------------------------------------------

int RockchipMppH264DecoderImpl::initMppContext() {
  // 1. Create context + MPI vtable.
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_api_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] mpp_create failed: " << ret;
    return ret;
  }

  // 2. Decoder-specific knobs BEFORE mpp_init:
  //    - parser_split_mode=1 lets MPP accept Annex-B streams where a
  //      single packet may contain multiple NAL units (which is how
  //      libwebrtc hands us H.264 from the depacketizer).
  RK_U32 split_mode = 1;
  ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_PARSER_SPLIT_MODE,
                          &split_mode);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] SET_PARSER_SPLIT_MODE failed: " << ret;
    return ret;
  }

  // 3. IO timeouts: block on input (so decode_put_packet only returns when
  //    the parser has actually consumed the buffer); 100ms on output (so
  //    decode_get_frame doesn't pin the decoder thread on stalls).
  RK_S64 in_timeout = MPP_POLL_BLOCK;
  ret = mpp_api_->control(mpp_ctx_, MPP_SET_INPUT_TIMEOUT, &in_timeout);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] SET_INPUT_TIMEOUT failed: " << ret;
    return ret;
  }
  RK_S64 out_timeout = 100; // ms
  ret = mpp_api_->control(mpp_ctx_, MPP_SET_OUTPUT_TIMEOUT, &out_timeout);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] SET_OUTPUT_TIMEOUT failed: " << ret;
    return ret;
  }

  // 4. Initialize for AVC decode.
  ret = mpp_init(mpp_ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingAVC);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] mpp_init(DEC, AVC) failed: " << ret;
    return ret;
  }

  // 5. Allocate a long-lived input MppPacket that we re-aim at each
  //    EncodedImage's bytes via mpp_packet_set_data/length.
  ret = mpp_packet_init(&packet_, nullptr, 0);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] mpp_packet_init failed: " << ret;
    return ret;
  }

  RTC_LOG(LS_INFO) << "[mpp-dec] context ready (AVC, split=1, out=100ms)";
  return 0;
}

bool RockchipMppH264DecoderImpl::drainOneFrame(uint32_t rtp_timestamp) {
  MppFrame frame = nullptr;
  MPP_RET ret = mpp_api_->decode_get_frame(mpp_ctx_, &frame);
  if (ret == MPP_ERR_TIMEOUT || !frame) {
    return false; // nothing to do this round
  }
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] decode_get_frame failed: " << ret;
    if (frame)
      mpp_frame_deinit(&frame);
    return false;
  }

  // info-change handshake: first frame after Configure (or after a
  // resolution shift) carries dimensions only; we provide buffer storage,
  // signal info_change_ready, MPP retries decoding into our group.
  if (mpp_frame_get_info_change(frame)) {
    RK_U32 w = mpp_frame_get_width(frame);
    RK_U32 h = mpp_frame_get_height(frame);
    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
    RK_U32 buf_size = mpp_frame_get_buf_size(frame);
    RTC_LOG(LS_INFO) << "[mpp-dec] info change " << w << "x" << h << " stride "
                     << hor_stride << "x" << ver_stride << " buf_size "
                     << buf_size;
    width_ = static_cast<int>(w);
    height_ = static_cast<int>(h);

    if (frm_grp_) {
      // Resolution shift: tear down old group, build a new one sized for
      // the new buf_size. MPP will pull buffers back as it finishes its
      // pending frames.
      mpp_buffer_group_clear(frm_grp_);
    } else {
      MPP_RET grp_ret = mpp_buffer_group_get_internal(
          &frm_grp_, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
      if (grp_ret) {
        RTC_LOG(LS_ERROR) << "[mpp-dec] buffer_group_get_internal failed: "
                          << grp_ret;
        mpp_frame_deinit(&frame);
        return false;
      }
      // 24 buffers = mpi_dec_test default; covers worst-case reordering
      // for B-frames-heavy AVC. WebRTC streams are typically GOP-only
      // (no B), so we usually consume far fewer.
      mpp_buffer_group_limit_config(frm_grp_, buf_size, 24);
      MPP_RET set_ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_EXT_BUF_GROUP,
                                          frm_grp_);
      if (set_ret) {
        RTC_LOG(LS_ERROR) << "[mpp-dec] MPP_DEC_SET_EXT_BUF_GROUP failed: "
                          << set_ret;
        mpp_frame_deinit(&frame);
        return false;
      }
    }
    MPP_RET ack_ret = mpp_api_->control(mpp_ctx_,
                                        MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
    if (ack_ret) {
      RTC_LOG(LS_ERROR) << "[mpp-dec] MPP_DEC_SET_INFO_CHANGE_READY failed: "
                        << ack_ret;
    }
    mpp_frame_deinit(&frame);
    return true; // try next get_frame — real frame may follow immediately
  }

  // Real decoded frame.
  RK_U32 err_info = mpp_frame_get_errinfo(frame);
  RK_U32 discard = mpp_frame_get_discard(frame);
  if (err_info || discard) {
    RTC_LOG(LS_VERBOSE) << "[mpp-dec] frame err=" << err_info
                        << " discard=" << discard;
    mpp_frame_deinit(&frame);
    return true; // skip but keep draining
  }

  RK_U32 fw = mpp_frame_get_width(frame);
  RK_U32 fh = mpp_frame_get_height(frame);
  RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
  RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
  MppBuffer buf = mpp_frame_get_buffer(frame);
  if (!buf || fw == 0 || fh == 0) {
    mpp_frame_deinit(&frame);
    return true;
  }

  const uint8_t *src = static_cast<const uint8_t *>(mpp_buffer_get_ptr(buf));
  if (!src) {
    mpp_frame_deinit(&frame);
    return true;
  }
  mpp_buffer_sync_begin(buf);

  // NV12 → I420 via libyuv. The pool recycles I420Buffers so we don't
  // allocate per-frame.
  webrtc::scoped_refptr<webrtc::I420Buffer> i420 =
      buffer_pool_.CreateI420Buffer(static_cast<int>(fw),
                                    static_cast<int>(fh));
  if (!i420) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] CreateI420Buffer failed " << fw << "x"
                      << fh;
    mpp_buffer_sync_end(buf);
    mpp_frame_deinit(&frame);
    return true;
  }

  const uint8_t *src_y = src;
  const uint8_t *src_uv = src + hor_stride * ver_stride;
  int rc = libyuv::NV12ToI420(
      src_y, static_cast<int>(hor_stride), src_uv, static_cast<int>(hor_stride),
      i420->MutableDataY(), i420->StrideY(), i420->MutableDataU(),
      i420->StrideU(), i420->MutableDataV(), i420->StrideV(),
      static_cast<int>(fw), static_cast<int>(fh));
  mpp_buffer_sync_end(buf);
  if (rc) {
    RTC_LOG(LS_WARNING) << "[mpp-dec] libyuv NV12ToI420 rc=" << rc;
  }

  VideoFrame decoded_frame =
      VideoFrame::Builder()
          .set_video_frame_buffer(i420)
          .set_rtp_timestamp(rtp_timestamp)
          .build();

  mpp_frame_deinit(&frame);

  decoded_complete_callback_->Decoded(decoded_frame, std::nullopt,
                                      std::nullopt);
  return true;
}

void RockchipMppH264DecoderImpl::teardownMppContext() {
  if (packet_) {
    mpp_packet_deinit(&packet_);
    packet_ = nullptr;
  }
  if (mpp_ctx_) {
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    mpp_api_ = nullptr;
  }
  if (frm_grp_) {
    mpp_buffer_group_put(frm_grp_);
    frm_grp_ = nullptr;
  }
}

} // namespace webrtc
