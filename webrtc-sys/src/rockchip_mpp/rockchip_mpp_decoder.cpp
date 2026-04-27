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
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "api/video/i420_buffer.h"
#include "api/video/nv12_buffer.h"
#include "api/video/video_frame.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/logging.h"

extern "C" {
#include "mpp_err.h"
#include "rk_mpi_cmd.h"
}

namespace webrtc {

namespace {
const char *codingTypeName(MppCodingType c) {
  switch (c) {
    case MPP_VIDEO_CodingAVC:  return "H264";
    case MPP_VIDEO_CodingHEVC: return "H265";
    case MPP_VIDEO_CodingVP8:  return "VP8";
    case MPP_VIDEO_CodingVP9:  return "VP9";
    default:                   return "Unknown";
  }
}
} // namespace

RockchipMppVideoDecoderImpl::RockchipMppVideoDecoderImpl(
    const SdpVideoFormat &format, MppCodingType coding)
    : format_(format), coding_(coding),
      buffer_pool_(/*zero_initialize=*/false, 16) {
  RTC_LOG(LS_INFO) << "[mpp-dec] ctor codec=" << codingTypeName(coding_);
}

RockchipMppVideoDecoderImpl::~RockchipMppVideoDecoderImpl() { Release(); }

bool RockchipMppVideoDecoderImpl::Configure(const Settings & /*settings*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_)
    return true;
  std::fprintf(stderr, "[mpp-dec] Configure() — opening MPP context\n");
  int rc = initMppContext();
  if (rc != 0) {
    std::fprintf(stderr, "[mpp-dec] initMppContext failed rc=%d\n", rc);
    RTC_LOG(LS_ERROR) << "[mpp-dec] initMppContext failed: " << rc;
    teardownMppContext();
    return false;
  }
  std::fprintf(stderr, "[mpp-dec] Configure OK\n");
  initialized_ = true;
  return true;
}

int32_t RockchipMppVideoDecoderImpl::Release() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    teardownMppContext();
    initialized_ = false;
  }
  decoded_complete_callback_ = nullptr;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipMppVideoDecoderImpl::RegisterDecodeCompleteCallback(
    DecodedImageCallback *callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  decoded_complete_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int32_t RockchipMppVideoDecoderImpl::Decode(const EncodedImage &input_image,
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
  // Log first packet only — subsequent packets at WebRTC ingress rate
  // (~30/s) would flood stderr.
  static thread_local bool logged_first_decode = false;
  if (!logged_first_decode) {
    std::fprintf(stderr, "[mpp-dec] Decode() first packet size=%zu bytes\n",
                 input_image.size());
    logged_first_decode = true;
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

  // 3. Drain frames. Typical case: 1 packet in → 1 frame out, so we exit
  //    immediately after delivering that frame. info_change resolves
  //    without a frame payload — loop again to grab the actual frame
  //    queued behind it. Cap the iterations as a sanity guard against
  //    unexpected MPP states.
  for (int i = 0; i < 4; ++i) {
    DrainResult rc = drainOneFrame(input_image.RtpTimestamp());
    if (rc == DrainResult::kFrame || rc == DrainResult::kNothing) {
      break;
    }
    // info_change → retry to fetch the real frame
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

VideoDecoder::DecoderInfo RockchipMppVideoDecoderImpl::GetDecoderInfo() const {
  DecoderInfo info;
  info.implementation_name =
      std::string("RockchipMpp_") + codingTypeName(coding_);
  info.is_hardware_accelerated = true;
  return info;
}

// ---- private helpers ---------------------------------------------------

int RockchipMppVideoDecoderImpl::initMppContext() {
  // 1. Create context + MPI vtable.
  MPP_RET ret = mpp_create(&mpp_ctx_, &mpp_api_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] mpp_create failed: " << ret;
    return ret;
  }

  // 2. Decoder-specific knobs BEFORE mpp_init:
  //    - parser_split_mode=1 lets MPP accept Annex-B byte-streams where
  //      a single packet may contain multiple NAL units (H.264 and H.265
  //      from libwebrtc's depacketizer). VP8/VP9 frames are already
  //      complete-per-packet, no split needed.
  if (coding_ == MPP_VIDEO_CodingAVC || coding_ == MPP_VIDEO_CodingHEVC) {
    RK_U32 split_mode = 1;
    ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_PARSER_SPLIT_MODE,
                            &split_mode);
    if (ret) {
      RTC_LOG(LS_ERROR) << "[mpp-dec] SET_PARSER_SPLIT_MODE failed: " << ret;
      return ret;
    }
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

  // 4. Initialize for the configured codec.
  ret = mpp_init(mpp_ctx_, MPP_CTX_DEC, coding_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] mpp_init(DEC, " << codingTypeName(coding_)
                      << ") failed: " << ret;
    return ret;
  }

  // 5. Allocate a long-lived input MppPacket that we re-aim at each
  //    EncodedImage's bytes via mpp_packet_set_data/length.
  ret = mpp_packet_init(&packet_, nullptr, 0);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] mpp_packet_init failed: " << ret;
    return ret;
  }

  RTC_LOG(LS_INFO) << "[mpp-dec] context ready (" << codingTypeName(coding_)
                   << ", out=100ms)";
  return 0;
}

RockchipMppVideoDecoderImpl::DrainResult
RockchipMppVideoDecoderImpl::drainOneFrame(uint32_t rtp_timestamp) {
  MppFrame frame = nullptr;
  MPP_RET ret = mpp_api_->decode_get_frame(mpp_ctx_, &frame);
  if (ret == MPP_ERR_TIMEOUT || !frame) {
    return DrainResult::kNothing;
  }
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] decode_get_frame failed: " << ret;
    if (frame)
      mpp_frame_deinit(&frame);
    return DrainResult::kNothing;
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

    if (!frm_grp_) {
      MPP_RET grp_ret = mpp_buffer_group_get_internal(
          &frm_grp_, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
      if (grp_ret) {
        RTC_LOG(LS_ERROR) << "[mpp-dec] buffer_group_get_internal failed: "
                          << grp_ret;
        mpp_frame_deinit(&frame);
        return DrainResult::kNothing;
      }
      MPP_RET set_ret = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_EXT_BUF_GROUP,
                                          frm_grp_);
      if (set_ret) {
        RTC_LOG(LS_ERROR) << "[mpp-dec] MPP_DEC_SET_EXT_BUF_GROUP failed: "
                          << set_ret;
        mpp_frame_deinit(&frame);
        return DrainResult::kNothing;
      }
    }
    // Re-set the per-buffer size cap on every info_change. Initial cap
    // was sized for the first frame's buf_size; simulcast layer upgrades
    // grow the frame (e.g. 92 KB → 471 KB → 1843 KB at 180x320 → 360x640
    // → 720x1280) and mpp_buffer_create then fails with "reach group size
    // limit", which freezes decoding while audio keeps flowing — a really
    // nasty failure mode because the group LOOKS fine. 24 buffers =
    // mpi_dec_test default; covers worst-case reordering for B-frames-
    // heavy AVC. WebRTC streams are typically GOP-only (no B), so we
    // usually consume far fewer.
    //
    // Also clear the group: with limit_config alone, MPP keeps the old
    // smaller buffers in the slot table and won't reallocate them even
    // when the new larger size is permitted. Without clearing, simulcast
    // upgrades stall after the first higher-layer frame slot is requested.
    // The earlier worry that clear races with MPP's worker thread turned
    // out to be wrong — that segfault was actually about the EXIT path,
    // which is now properly handled in teardownMppContext via
    // MPP_DEC_SET_EXT_BUF_GROUP=NULL + reset.
    mpp_buffer_group_limit_config(frm_grp_, buf_size, 24);
    mpp_buffer_group_clear(frm_grp_);
    MPP_RET ack_ret = mpp_api_->control(mpp_ctx_,
                                        MPP_DEC_SET_INFO_CHANGE_READY, nullptr);
    if (ack_ret) {
      RTC_LOG(LS_ERROR) << "[mpp-dec] MPP_DEC_SET_INFO_CHANGE_READY failed: "
                        << ack_ret;
    }
    mpp_frame_deinit(&frame);
    return DrainResult::kInfoChange;
  }

  // Real decoded frame.
  RK_U32 err_info = mpp_frame_get_errinfo(frame);
  RK_U32 discard = mpp_frame_get_discard(frame);
  if (err_info || discard) {
    RTC_LOG(LS_VERBOSE) << "[mpp-dec] frame err=" << err_info
                        << " discard=" << discard;
    mpp_frame_deinit(&frame);
    // Treat as nothing-delivered so the loop bails — we don't expect more
    // frames immediately after a discarded one.
    return DrainResult::kNothing;
  }

  RK_U32 fw = mpp_frame_get_width(frame);
  RK_U32 fh = mpp_frame_get_height(frame);
  RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
  RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
  MppBuffer buf = mpp_frame_get_buffer(frame);
  if (!buf || fw == 0 || fh == 0) {
    mpp_frame_deinit(&frame);
    return DrainResult::kNothing;
  }

  const uint8_t *src = static_cast<const uint8_t *>(mpp_buffer_get_ptr(buf));
  if (!src) {
    mpp_frame_deinit(&frame);
    return DrainResult::kNothing;
  }
  size_t mpp_buf_bytes = mpp_buffer_get_size(buf);
  size_t y_plane_bytes = static_cast<size_t>(hor_stride) * ver_stride;
  size_t needed_bytes = y_plane_bytes + (y_plane_bytes / 2);
  if (mpp_buf_bytes < needed_bytes) {
    // Simulcast layer upgrade race: MPP delivers a frame whose stride/size
    // doesn't fit the buffer it allocated from our group. The next
    // info_change resizes the group; just drop this one.
    RTC_LOG(LS_VERBOSE) << "[mpp-dec] skip frame: buf=" << mpp_buf_bytes
                        << " < need=" << needed_bytes;
    mpp_frame_deinit(&frame);
    return DrainResult::kNothing;
  }
  mpp_buffer_sync_begin(buf);

  // NV12 stride-strip into a fresh NV12Buffer. We used to convert to I420
  // via libyuv NV12ToI420 here, but the SDK FFI's downstream conversion
  // pipeline (livekit-ffi/server/colorcvt) only supports I420→NV12 if the
  // source is already NV12 — going through I420 forced a round trip
  // (NV12→I420 here, I420→NV12 in the consumer's DRM display path), each
  // ~1.4 MB at 720p × 30 fps = ~42 MB/s of pointless memory traffic per
  // direction. Returning NV12 lets the FFI's cvt_nv12 path fall through to
  // a single nv12_copy memcpy and the consumer skip the second conversion.
  //
  // MPP's hor_stride / ver_stride are usually larger than width / height
  // (16-aligned at minimum), so we still copy row-by-row to strip stride
  // padding into the NV12Buffer's tight (stride == width) layout.
  webrtc::scoped_refptr<webrtc::NV12Buffer> nv12 = webrtc::NV12Buffer::Create(
      static_cast<int>(fw), static_cast<int>(fh));
  if (!nv12) {
    RTC_LOG(LS_ERROR) << "[mpp-dec] NV12Buffer::Create failed " << fw << "x"
                      << fh;
    mpp_buffer_sync_end(buf);
    mpp_frame_deinit(&frame);
    return DrainResult::kNothing;
  }

  const uint8_t *src_y = src;
  const uint8_t *src_uv = src + static_cast<size_t>(hor_stride) * ver_stride;
  uint8_t *dst_y = nv12->MutableDataY();
  uint8_t *dst_uv = nv12->MutableDataUV();
  int dst_stride_y = nv12->StrideY();
  int dst_stride_uv = nv12->StrideUV();
  for (int row = 0; row < static_cast<int>(fh); ++row) {
    std::memcpy(dst_y + row * dst_stride_y, src_y + row * hor_stride, fw);
  }
  int chroma_h = (static_cast<int>(fh) + 1) / 2;
  for (int row = 0; row < chroma_h; ++row) {
    std::memcpy(dst_uv + row * dst_stride_uv, src_uv + row * hor_stride, fw);
  }
  mpp_buffer_sync_end(buf);

  VideoFrame decoded_frame =
      VideoFrame::Builder()
          .set_video_frame_buffer(nv12)
          .set_rtp_timestamp(rtp_timestamp)
          .build();

  mpp_frame_deinit(&frame);

  decoded_complete_callback_->Decoded(decoded_frame, std::nullopt,
                                      std::nullopt);
  return DrainResult::kFrame;
}

void RockchipMppVideoDecoderImpl::teardownMppContext() {
  // Drain MPP first so its internal queues release references to our
  // external buffer group + input packet. Without this, mpp_destroy
  // leaves dangling references that libmpp's atexit service handlers
  // (mpp_buffer_service_deinit, mpp_meta_srv_deinit) later try to clean
  // up — touching freed memory and segfaulting at process exit.
  if (mpp_api_ && mpp_ctx_) {
    mpp_api_->reset(mpp_ctx_);
  }
  // Detach buffer group from the decoder before destroy so mpp_destroy
  // doesn't keep references to a group we're about to put. mpi_dec_test
  // does the clear+put dance via dec_buf_mgr_deinit; we inline it here.
  if (mpp_api_ && mpp_ctx_ && frm_grp_) {
    mpp_api_->control(mpp_ctx_, MPP_DEC_SET_EXT_BUF_GROUP, nullptr);
  }
  if (frm_grp_) {
    mpp_buffer_group_clear(frm_grp_);
  }
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
