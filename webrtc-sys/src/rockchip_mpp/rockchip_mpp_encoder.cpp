/*
 * Copyright 2026 LiveKit
 * Licensed under the Apache License, Version 2.0.
 *
 * Phase 6.1.4 — real Rockchip MPP H.264 encode path:
 *   InitEncode  : mpp_create + mpp_init(MPP_CTX_ENC, AVC) + MppEncCfg
 *                 (size/fmt=NV12/fps/rc=cbr/bps/gop) + DRM buffer group
 *                 + sps/pps cached via MPP_ENC_GET_HDR_SYNC
 *   Encode      : I420 → NV12 software convert into MppBuffer →
 *                 encode_put_frame → encode_get_packet → EncodedImage →
 *                 OnEncodedImage(). SPS/PPS prepended on each IDR.
 *   SetRates    : forwards bps_target/fps to MPP via MPP_ENC_SET_CFG.
 *
 * RGA zero-copy + simulcast → Phase 6.1.5. Decoder → Phase 6.2.
 */

// MODULE_TAG must be defined BEFORE including mpp_buffer.h — its
// mpp_buffer_get / mpp_buffer_get_ptr macros reference it for caller
// tagging on the kernel side.
#define MODULE_TAG "livekit_mpp_enc"

#include "rockchip_mpp_encoder.h"

#include <cstring>

#include "api/video/i420_buffer.h"
#include "api/video/video_frame_buffer.h"
#include "modules/video_coding/codecs/h264/include/h264_globals.h"
#include "modules/video_coding/include/video_codec_interface.h"
#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/logging.h"
#include "rtc_base/time_utils.h"
#include "third_party/libyuv/include/libyuv/convert_from.h"

extern "C" {
// rockchip_mpp_encoder.h already pulls in rk_mpi.h / mpp_buffer.h / ...
// We just need the bits used only inside this TU here:
#include "mpp_err.h"
#include "mpp_meta.h"
}

namespace webrtc {

namespace {

constexpr int kAlign = 16;
inline int AlignUp(int x, int a) { return (x + a - 1) & ~(a - 1); }

// I420 → NV12 (semi-planar): Y plane copied as-is (with stride padding),
// U/V interleaved into a single plane. Output written into the MppBuffer
// that backs an MppFrame at the given hor/ver strides.
//
// Backed by libyuv::I420ToNV12 which is NEON-vectorized on aarch64 — at
// 720p the previous hand-rolled scalar interleave loop was ~10x slower
// per frame.
void I420ToNV12(const uint8_t *src_y, int src_stride_y, const uint8_t *src_u,
                int src_stride_u, const uint8_t *src_v, int src_stride_v,
                int width, int height, uint8_t *dst, int hor_stride,
                int ver_stride) {
  uint8_t *dst_y = dst;
  uint8_t *dst_uv = dst + static_cast<size_t>(hor_stride) * ver_stride;
  ::libyuv::I420ToNV12(src_y, src_stride_y, src_u, src_stride_u, src_v,
                       src_stride_v, dst_y, hor_stride, dst_uv, hor_stride,
                       width, height);
}

// Look at the first NAL unit header to decide if a packet starts with an
// IDR. RV1126B emits Annex-B (00 00 00 01 / 00 00 01) start codes.
bool PacketStartsWithIdr(const uint8_t *data, size_t len) {
  if (len < 5)
    return false;
  size_t i = 0;
  // Skip start code
  if (data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1) {
    i = 4;
  } else if (data[0] == 0 && data[1] == 0 && data[2] == 1) {
    i = 3;
  } else {
    return false;
  }
  if (i >= len)
    return false;
  uint8_t nal_type = data[i] & 0x1F;
  // 5 = IDR slice; 7 = SPS; 8 = PPS — both bracket IDR for keyframe payloads.
  return nal_type == 5 || nal_type == 7 || nal_type == 8;
}

} // namespace

RockchipMppH264EncoderImpl::RockchipMppH264EncoderImpl(
    const SdpVideoFormat &format)
    : format_(format) {
  RTC_LOG(LS_INFO) << "[mpp] RockchipMppH264EncoderImpl ctor";
}

RockchipMppH264EncoderImpl::~RockchipMppH264EncoderImpl() { Release(); }

int RockchipMppH264EncoderImpl::InitEncode(const VideoCodec *codec_settings,
                                           const Settings & /*settings*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!codec_settings) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  width_ = codec_settings->width;
  height_ = codec_settings->height;
  fps_ = codec_settings->maxFramerate ? codec_settings->maxFramerate : 30;
  target_bps_ = static_cast<int>(codec_settings->startBitrate * 1000);
  if (target_bps_ <= 0)
    target_bps_ = 1'500'000;
  gop_len_ = fps_ * 2;
  hor_stride_ = AlignUp(width_, kAlign);
  ver_stride_ = AlignUp(height_, kAlign);
  RTC_LOG(LS_INFO) << "[mpp] InitEncode " << width_ << "x" << height_ << " ("
                   << hor_stride_ << "x" << ver_stride_ << ")@" << fps_
                   << "fps target_bps=" << target_bps_ << " gop=" << gop_len_;

  int rc = initMppContext(width_, height_, fps_, target_bps_, gop_len_);
  if (rc != 0) {
    RTC_LOG(LS_ERROR) << "[mpp] initMppContext failed: " << rc;
    teardownMppContext();
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  initialized_ = true;
  return WEBRTC_VIDEO_CODEC_OK;
}

int RockchipMppH264EncoderImpl::Release() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (initialized_) {
    teardownMppContext();
    initialized_ = false;
  }
  encoded_callback_ = nullptr;
  return WEBRTC_VIDEO_CODEC_OK;
}

int RockchipMppH264EncoderImpl::RegisterEncodeCompleteCallback(
    EncodedImageCallback *callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  encoded_callback_ = callback;
  return WEBRTC_VIDEO_CODEC_OK;
}

int RockchipMppH264EncoderImpl::Encode(
    const VideoFrame &frame,
    const std::vector<VideoFrameType> *frame_types) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !mpp_ctx_ || !mpp_api_) {
    return WEBRTC_VIDEO_CODEC_UNINITIALIZED;
  }
  if (!encoded_callback_) {
    return WEBRTC_VIDEO_CODEC_OK; // nobody listening yet
  }

  // 1. Pull I420 view of the source frame and copy/convert into the
  //    pre-allocated MppBuffer at hor_stride/ver_stride.
  webrtc::scoped_refptr<webrtc::I420BufferInterface> i420 =
      frame.video_frame_buffer()->ToI420();
  if (!i420) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  if (i420->width() != width_ || i420->height() != height_) {
    RTC_LOG(LS_WARNING) << "[mpp] frame size " << i420->width() << "x"
                        << i420->height() << " != configured " << width_ << "x"
                        << height_ << " — skipping";
    return WEBRTC_VIDEO_CODEC_OK;
  }

  uint8_t *dst = static_cast<uint8_t *>(mpp_buffer_get_ptr(frm_buf_));
  if (!dst)
    return WEBRTC_VIDEO_CODEC_ERROR;
  mpp_buffer_sync_begin(frm_buf_);
  I420ToNV12(i420->DataY(), i420->StrideY(), i420->DataU(), i420->StrideU(),
             i420->DataV(), i420->StrideV(), width_, height_, dst, hor_stride_,
             ver_stride_);
  mpp_buffer_sync_end(frm_buf_);

  // 2. Force-IDR if libwebrtc requested a keyframe.
  bool want_keyframe = false;
  if (frame_types) {
    for (auto t : *frame_types) {
      if (t == VideoFrameType::kVideoFrameKey) {
        want_keyframe = true;
        break;
      }
    }
  }
  if (want_keyframe) {
    MPP_RET ret = mpp_api_->control(mpp_ctx_, MPP_ENC_SET_IDR_FRAME, nullptr);
    if (ret) {
      RTC_LOG(LS_WARNING) << "[mpp] MPP_ENC_SET_IDR_FRAME failed: " << ret;
    }
  }

  // 3. Build MppFrame around our input buffer.
  MppFrame mpp_frame = nullptr;
  if (mpp_frame_init(&mpp_frame)) {
    return WEBRTC_VIDEO_CODEC_ERROR;
  }
  mpp_frame_set_width(mpp_frame, width_);
  mpp_frame_set_height(mpp_frame, height_);
  mpp_frame_set_hor_stride(mpp_frame, hor_stride_);
  mpp_frame_set_ver_stride(mpp_frame, ver_stride_);
  mpp_frame_set_fmt(mpp_frame, MPP_FMT_YUV420SP);
  mpp_frame_set_eos(mpp_frame, 0);
  mpp_frame_set_buffer(mpp_frame, frm_buf_);

  // 4. Pre-bind our reusable output packet to the frame's metadata.
  //    NOTE: encode_get_packet(&packet) below will re-assign this same
  //    variable; MPP transfers ownership through it. Deiniting both the
  //    pre-bind handle AND the post-get handle is a double-free (per
  //    mpi_enc_test.c which uses one variable for both phases).
  MppPacket packet = nullptr;
  mpp_packet_init_with_buffer(&packet, pkt_buf_);
  mpp_packet_set_length(packet, 0);
  MppMeta meta = mpp_frame_get_meta(mpp_frame);
  mpp_meta_set_packet(meta, KEY_OUTPUT_PACKET, packet);

  // 5. Submit + drain. Output timeout was set to MPP_POLL_BLOCK at init,
  //    so encode_get_packet returns when a NAL is ready (or EOS).
  MPP_RET ret = mpp_api_->encode_put_frame(mpp_ctx_, mpp_frame);
  mpp_frame_deinit(&mpp_frame);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] encode_put_frame failed: " << ret;
    mpp_packet_deinit(&packet);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  ret = mpp_api_->encode_get_packet(mpp_ctx_, &packet);
  if (ret || !packet) {
    RTC_LOG(LS_ERROR) << "[mpp] encode_get_packet failed: " << ret;
    if (packet)
      mpp_packet_deinit(&packet);
    return WEBRTC_VIDEO_CODEC_ERROR;
  }

  const uint8_t *pkt_pos =
      static_cast<const uint8_t *>(mpp_packet_get_pos(packet));
  size_t pkt_len = mpp_packet_get_length(packet);

  // 6. Determine intra/inter via meta first, falling back to NAL parse.
  bool is_idr = false;
  if (mpp_packet_has_meta(packet)) {
    MppMeta out_meta = mpp_packet_get_meta(packet);
    RK_S32 intra_flag = 0;
    if (mpp_meta_get_s32(out_meta, KEY_OUTPUT_INTRA, &intra_flag) == MPP_OK) {
      is_idr = (intra_flag != 0);
    }
  }
  if (!is_idr) {
    is_idr = PacketStartsWithIdr(pkt_pos, pkt_len) || want_keyframe;
  }

  // 7. Build EncodedImage. Prepend cached SPS/PPS on every IDR — libwebrtc's
  //    H.264 RTP packetizer / depacketizer expects IDR access units to
  //    carry their parameter sets so a late-joining receiver can decode.
  std::vector<uint8_t> payload;
  payload.reserve(pkt_len + (is_idr ? hdr_pps_sps_.size() : 0));
  if (is_idr && !hdr_pps_sps_.empty() && pkt_len > 4) {
    // Avoid double-prepending if MPP already emitted SPS/PPS in front of IDR.
    bool already_has_sps = false;
    if (pkt_len >= 5) {
      size_t i = (pkt_pos[2] == 1) ? 3 : 4;
      if (i < pkt_len) {
        uint8_t nal = pkt_pos[i] & 0x1F;
        already_has_sps = (nal == 7 || nal == 8);
      }
    }
    if (!already_has_sps) {
      payload.insert(payload.end(), hdr_pps_sps_.begin(), hdr_pps_sps_.end());
    }
  }
  payload.insert(payload.end(), pkt_pos, pkt_pos + pkt_len);

  EncodedImage encoded;
  encoded.SetEncodedData(EncodedImageBuffer::Create(payload.data(),
                                                    payload.size()));
  encoded._encodedWidth = width_;
  encoded._encodedHeight = height_;
  encoded.SetRtpTimestamp(frame.rtp_timestamp());
  encoded.capture_time_ms_ = frame.render_time_ms();
  encoded._frameType = is_idr ? VideoFrameType::kVideoFrameKey
                              : VideoFrameType::kVideoFrameDelta;

  // Parse to populate qp_ for stats / pacer (best-effort; ignore failures).
  h264_parser_.ParseBitstream(encoded);
  if (auto qp_opt = h264_parser_.GetLastSliceQp()) {
    encoded.qp_ = *qp_opt;
  }

  CodecSpecificInfo codec_specific;
  codec_specific.codecType = kVideoCodecH264;
  codec_specific.codecSpecific.H264.packetization_mode =
      H264PacketizationMode::NonInterleaved;
  codec_specific.codecSpecific.H264.idr_frame = is_idr;
  codec_specific.codecSpecific.H264.base_layer_sync = false;
  codec_specific.codecSpecific.H264.temporal_idx = kNoTemporalIdx;

  EncodedImageCallback::Result cb_result =
      encoded_callback_->OnEncodedImage(encoded, &codec_specific);

  // Single deinit — the pre-bind handle and the get_packet handle are the
  // same MppPacket; deinit twice would tank ref counts (cf. [33.x] fix).
  mpp_packet_deinit(&packet);

  if (cb_result.error != EncodedImageCallback::Result::OK) {
    RTC_LOG(LS_WARNING) << "[mpp] OnEncodedImage error: "
                        << static_cast<int>(cb_result.error);
  }

  return WEBRTC_VIDEO_CODEC_OK;
}

void RockchipMppH264EncoderImpl::SetRates(
    const RateControlParameters &parameters) {
  std::lock_guard<std::mutex> lock(mutex_);
  int new_bps = static_cast<int>(parameters.bitrate.get_sum_bps());
  int new_fps = static_cast<int>(parameters.framerate_fps);
  if (new_fps <= 0)
    new_fps = fps_;
  if (new_bps <= 0)
    return;
  if (new_bps == target_bps_ && new_fps == fps_) {
    return; // no change
  }
  target_bps_ = new_bps;
  fps_ = new_fps;
  if (!initialized_)
    return;
  int rc = applyRateControl();
  if (rc) {
    RTC_LOG(LS_WARNING) << "[mpp] applyRateControl failed: " << rc;
  } else {
    RTC_LOG(LS_VERBOSE) << "[mpp] SetRates → bps=" << target_bps_
                        << " fps=" << fps_;
  }
}

VideoEncoder::EncoderInfo
RockchipMppH264EncoderImpl::GetEncoderInfo() const {
  EncoderInfo info;
  info.implementation_name = "RockchipMpp_H264";
  info.is_hardware_accelerated = true;
  info.supports_native_handle = false;
  info.supports_simulcast = false;
  return info;
}

// ---- private helpers ---------------------------------------------------

int RockchipMppH264EncoderImpl::initMppContext(int width, int height, int fps,
                                               int target_bps, int gop_len) {
  // 1. Allocate DRM buffer group + reusable input/output buffers.
  //    Frame size for NV12 = hor_stride * ver_stride * 3/2.
  size_t frame_size = static_cast<size_t>(hor_stride_) * ver_stride_ * 3 / 2;
  size_t pkt_size = frame_size; // upper bound on a single encoded NAL
  MPP_RET ret = mpp_buffer_group_get_internal(
      &buf_grp_, MPP_BUFFER_TYPE_DRM | MPP_BUFFER_FLAGS_CACHABLE);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] buffer_group_get_internal failed: " << ret;
    return ret;
  }
  ret = mpp_buffer_get(buf_grp_, &frm_buf_, frame_size);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] buffer_get(frm_buf) failed: " << ret;
    return ret;
  }
  ret = mpp_buffer_get(buf_grp_, &pkt_buf_, pkt_size);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] buffer_get(pkt_buf) failed: " << ret;
    return ret;
  }

  // 2. Create context + bind MPI.
  ret = mpp_create(&mpp_ctx_, &mpp_api_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] mpp_create failed: " << ret;
    return ret;
  }

  // 3. Block on encode_get_packet so we get one NAL per put_frame call.
  RK_S32 timeout = MPP_POLL_BLOCK;
  ret = mpp_api_->control(mpp_ctx_, MPP_SET_OUTPUT_TIMEOUT, &timeout);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] MPP_SET_OUTPUT_TIMEOUT failed: " << ret;
    return ret;
  }

  // 4. Initialize encoder for AVC.
  ret = mpp_init(mpp_ctx_, MPP_CTX_ENC, MPP_VIDEO_CodingAVC);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] mpp_init(ENC, AVC) failed: " << ret;
    return ret;
  }

  // 5. Build MppEncCfg and stuff it.
  ret = mpp_enc_cfg_init(&cfg_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] mpp_enc_cfg_init failed: " << ret;
    return ret;
  }
  ret = mpp_api_->control(mpp_ctx_, MPP_ENC_GET_CFG, cfg_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] MPP_ENC_GET_CFG failed: " << ret;
    return ret;
  }

  // PrePr (input format) — NV12 at the aligned strides we sized buffers for.
  mpp_enc_cfg_set_s32(cfg_, "prep:width", width);
  mpp_enc_cfg_set_s32(cfg_, "prep:height", height);
  mpp_enc_cfg_set_s32(cfg_, "prep:hor_stride", hor_stride_);
  mpp_enc_cfg_set_s32(cfg_, "prep:ver_stride", ver_stride_);
  mpp_enc_cfg_set_s32(cfg_, "prep:format", MPP_FMT_YUV420SP);

  // Rate control — CBR keyed off WebRTC's startBitrate.
  mpp_enc_cfg_set_s32(cfg_, "rc:mode", MPP_ENC_RC_MODE_CBR);
  mpp_enc_cfg_set_s32(cfg_, "rc:bps_target", target_bps);
  mpp_enc_cfg_set_s32(cfg_, "rc:bps_max", target_bps * 17 / 16);
  mpp_enc_cfg_set_s32(cfg_, "rc:bps_min", target_bps * 15 / 16);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_flex", 0);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_num", fps);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_denom", 1);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_flex", 0);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_num", fps);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_denom", 1);
  mpp_enc_cfg_set_s32(cfg_, "rc:gop", gop_len);

  // H.264-specific — Constrained Baseline (66) for max compatibility,
  // disable CABAC for low-latency, init QP via CBR engine.
  mpp_enc_cfg_set_s32(cfg_, "codec:type", MPP_VIDEO_CodingAVC);
  mpp_enc_cfg_set_s32(cfg_, "h264:profile", 66);
  mpp_enc_cfg_set_s32(cfg_, "h264:level", 31);
  mpp_enc_cfg_set_s32(cfg_, "h264:cabac_en", 0);
  mpp_enc_cfg_set_s32(cfg_, "h264:cabac_idc", 0);
  mpp_enc_cfg_set_s32(cfg_, "h264:trans8x8", 0);
  mpp_enc_cfg_set_s32(cfg_, "h264:qp_init", 26);
  mpp_enc_cfg_set_s32(cfg_, "h264:qp_max", 51);
  mpp_enc_cfg_set_s32(cfg_, "h264:qp_min", 10);
  mpp_enc_cfg_set_s32(cfg_, "h264:qp_max_i", 46);
  mpp_enc_cfg_set_s32(cfg_, "h264:qp_min_i", 18);

  ret = mpp_api_->control(mpp_ctx_, MPP_ENC_SET_CFG, cfg_);
  if (ret) {
    RTC_LOG(LS_ERROR) << "[mpp] MPP_ENC_SET_CFG failed: " << ret;
    return ret;
  }

  // 6. Cache SPS/PPS so we can prepend on every IDR.
  MppPacket hdr_pkt = nullptr;
  mpp_packet_init_with_buffer(&hdr_pkt, pkt_buf_);
  mpp_packet_set_length(hdr_pkt, 0);
  ret = mpp_api_->control(mpp_ctx_, MPP_ENC_GET_HDR_SYNC, hdr_pkt);
  if (ret == MPP_OK) {
    const uint8_t *hdr_pos =
        static_cast<const uint8_t *>(mpp_packet_get_pos(hdr_pkt));
    size_t hdr_len = mpp_packet_get_length(hdr_pkt);
    if (hdr_pos && hdr_len > 0) {
      hdr_pps_sps_.assign(hdr_pos, hdr_pos + hdr_len);
      RTC_LOG(LS_INFO) << "[mpp] cached SPS+PPS " << hdr_len << " bytes";
    }
  } else {
    RTC_LOG(LS_WARNING) << "[mpp] MPP_ENC_GET_HDR_SYNC failed: " << ret
                        << " (will rely on encoder-emitted SPS/PPS)";
  }
  mpp_packet_deinit(&hdr_pkt);

  return 0;
}

int RockchipMppH264EncoderImpl::applyRateControl() {
  if (!cfg_ || !mpp_api_ || !mpp_ctx_)
    return -1;
  mpp_enc_cfg_set_s32(cfg_, "rc:bps_target", target_bps_);
  mpp_enc_cfg_set_s32(cfg_, "rc:bps_max", target_bps_ * 17 / 16);
  mpp_enc_cfg_set_s32(cfg_, "rc:bps_min", target_bps_ * 15 / 16);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_in_num", fps_);
  mpp_enc_cfg_set_s32(cfg_, "rc:fps_out_num", fps_);
  return mpp_api_->control(mpp_ctx_, MPP_ENC_SET_CFG, cfg_);
}

void RockchipMppH264EncoderImpl::teardownMppContext() {
  if (mpp_ctx_) {
    mpp_destroy(mpp_ctx_);
    mpp_ctx_ = nullptr;
    mpp_api_ = nullptr;
  }
  if (cfg_) {
    mpp_enc_cfg_deinit(cfg_);
    cfg_ = nullptr;
  }
  if (frm_buf_) {
    mpp_buffer_put(frm_buf_);
    frm_buf_ = nullptr;
  }
  if (pkt_buf_) {
    mpp_buffer_put(pkt_buf_);
    pkt_buf_ = nullptr;
  }
  if (buf_grp_) {
    mpp_buffer_group_put(buf_grp_);
    buf_grp_ = nullptr;
  }
  hdr_pps_sps_.clear();
}

} // namespace webrtc
