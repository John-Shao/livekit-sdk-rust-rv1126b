/*
 * Copyright 2026 LiveKit
 *
 * Licensed under the Apache License, Version 2.0.
 *
 * Phase 6.1 SKELETON — wiring is in place but Encode() is a no-op stub
 * that returns WEBRTC_VIDEO_CODEC_ERROR. Real MPP API calls
 * (mpp_create / mpp_init / encode_put_frame / encode_get_packet)
 * are coming in Phase 6.1.4 — the factory's IsSupported() defaults
 * to false so we don't actually get selected over OpenH264 yet.
 */

#include "rockchip_mpp_encoder.h"

#include "modules/video_coding/include/video_error_codes.h"
#include "rtc_base/logging.h"

namespace webrtc {

RockchipMppH264EncoderImpl::RockchipMppH264EncoderImpl(
    const SdpVideoFormat &format)
    : format_(format) {
  RTC_LOG(LS_INFO) << "[mpp] RockchipMppH264EncoderImpl ctor (skeleton)";
}

RockchipMppH264EncoderImpl::~RockchipMppH264EncoderImpl() {
  Release();
}

int RockchipMppH264EncoderImpl::InitEncode(const VideoCodec *codec_settings,
                                           const Settings & /*settings*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!codec_settings) {
    return WEBRTC_VIDEO_CODEC_ERR_PARAMETER;
  }
  width_ = codec_settings->width;
  height_ = codec_settings->height;
  fps_ = codec_settings->maxFramerate;
  target_bps_ = static_cast<int>(codec_settings->startBitrate * 1000);
  gop_len_ = fps_ * 2;
  RTC_LOG(LS_INFO) << "[mpp] InitEncode " << width_ << "x" << height_ << "@"
                   << fps_ << "fps target_bps=" << target_bps_
                   << " gop=" << gop_len_;

  int rc = initMppContext(width_, height_, fps_, target_bps_, gop_len_);
  if (rc != 0) {
    RTC_LOG(LS_ERROR) << "[mpp] initMppContext failed: " << rc;
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
    const VideoFrame & /*frame*/,
    const std::vector<VideoFrameType> * /*frame_types*/) {
  // Phase 6.1 SKELETON: Real implementation will:
  //   1. Pull NV12 from frame's I420Buffer (or accept NV12 directly)
  //   2. Wrap in MppFrame (drm_buf-backed, ideally zero-copy via RGA)
  //   3. encode_put_frame()
  //   4. encode_get_packet() poll for output bitstream
  //   5. Hand bitstream to encoded_callback_->OnEncodedImage()
  RTC_LOG(LS_VERBOSE) << "[mpp] Encode (no-op stub)";
  return WEBRTC_VIDEO_CODEC_ERROR;
}

void RockchipMppH264EncoderImpl::SetRates(
    const RateControlParameters &parameters) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Future: forward target_bitrate_bps to MPP via MPP_ENC_SET_CFG / rc.bps_target
  RTC_LOG(LS_VERBOSE) << "[mpp] SetRates target_bps="
                      << parameters.bitrate.get_sum_bps()
                      << " framerate=" << parameters.framerate_fps;
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

int RockchipMppH264EncoderImpl::initMppContext(int /*width*/, int /*height*/,
                                               int /*fps*/, int /*target_bps*/,
                                               int /*gop_len*/) {
  // Phase 6.1 SKELETON: real implementation will:
  //   - mpp_create(&ctx, &mpi)
  //   - mpp_init(ctx, MPP_CTX_ENC, MPP_VIDEO_CodingAVC)
  //   - configure MppEncCfg (size, fps, rc=cbr, bps_target, gop, profile...)
  //   - mpp_buffer_group_get_internal(&frame_buf_group_, MPP_BUFFER_TYPE_DRM)
  //   - mpp_buffer_group_get_internal(&pkt_buf_group_, MPP_BUFFER_TYPE_DRM)
  //   - mpi->control(ctx, MPP_ENC_SET_CFG, cfg)
  // For now: return non-zero so Init fails and the factory falls back.
  RTC_LOG(LS_INFO) << "[mpp] initMppContext stub — returning non-zero so "
                      "OpenH264 keeps the H.264 slot until Phase 6.1.4";
  return -1; // intentional: pretend init failed
}

void RockchipMppH264EncoderImpl::teardownMppContext() {
  // Future: mpp_destroy + mpp_buffer_group_put on both groups
  mpp_ctx_ = nullptr;
  mpp_api_ = nullptr;
  frame_buf_group_ = nullptr;
  pkt_buf_group_ = nullptr;
}

} // namespace webrtc
