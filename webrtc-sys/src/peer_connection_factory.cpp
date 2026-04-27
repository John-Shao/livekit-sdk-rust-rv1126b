/*
 * Copyright 2025 LiveKit, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "livekit/peer_connection_factory.h"

#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include "api/audio_codecs/builtin_audio_decoder_factory.h"
#include "api/audio_codecs/builtin_audio_encoder_factory.h"
#include "api/audio/builtin_audio_processing_builder.h"
#include "api/create_modular_peer_connection_factory.h"
#include "api/environment/environment_factory.h"
#include "api/peer_connection_interface.h"
#include "api/rtc_error.h"
#include "api/enable_media.h"
#include "api/rtc_event_log/rtc_event_log_factory.h"
#include "api/task_queue/default_task_queue_factory.h"
#include "api/video_codecs/builtin_video_decoder_factory.h"
#include "api/video_codecs/builtin_video_encoder_factory.h"
#include "livekit/audio_device.h"
#include "livekit/audio_track.h"
#include "livekit/peer_connection.h"
#include "livekit/rtc_error.h"
#include "livekit/rtp_parameters.h"
#include "livekit/video_decoder_factory.h"
#include "livekit/video_encoder_factory.h"
#include "livekit/webrtc.h"
#include "rtc_base/thread.h"
#include "webrtc-sys/src/peer_connection.rs.h"
#include "webrtc-sys/src/peer_connection_factory.rs.h"

namespace livekit_ffi {

class PeerConnectionObserver;

PeerConnectionFactory::PeerConnectionFactory(
    std::shared_ptr<RtcRuntime> rtc_runtime)
    : rtc_runtime_(rtc_runtime),
    env_(webrtc::EnvironmentFactory().Create()) {
  RTC_LOG(LS_VERBOSE) << "PeerConnectionFactory::PeerConnectionFactory()";

  webrtc::PeerConnectionFactoryDependencies dependencies;
  dependencies.network_thread = rtc_runtime_->network_thread();
  dependencies.worker_thread = rtc_runtime_->worker_thread();
  dependencies.signaling_thread = rtc_runtime_->signaling_thread();
  dependencies.socket_factory = rtc_runtime_->network_thread()->socketserver();
  dependencies.event_log_factory = std::make_unique<webrtc::RtcEventLogFactory>();
  // TODO:
  // dependencies.trials = std::make_unique<webrtc::FieldTrialBasedConfig>();

  audio_device_ = rtc_runtime_->worker_thread()->BlockingCall([&] {
    return webrtc::make_ref_counted<livekit_ffi::AudioDevice>(env_);
  });

  dependencies.adm = audio_device_;

  dependencies.video_encoder_factory =
      std::move(std::make_unique<livekit_ffi::VideoEncoderFactory>());
  dependencies.video_decoder_factory =
      std::move(std::make_unique<livekit_ffi::VideoDecoderFactory>());
  dependencies.audio_encoder_factory = webrtc::CreateBuiltinAudioEncoderFactory();
  dependencies.audio_decoder_factory = webrtc::CreateBuiltinAudioDecoderFactory();
  // APM module gating — four concerns to balance, each disabled for a
  // specific reason on this board:
  //
  //   AGC1/AGC2: off. WebRTC's defaults turn both on, which silently undoes
  //     any explicit publish-side gain. The board's path is calibrated
  //     ES8389 PGA = +27 dB (numid=39/40 ALSA mixer) → 8× software gain in
  //     BoardLoopback → ~clipping; AGC then attenuates that right back down
  //     and the remote hears tinny "from the earpiece" audio.
  //
  //   AEC: off. AEC3 / AEC-mobile both need a "reverse stream" — a copy of
  //     the audio being played out the local speaker — so they can subtract
  //     it from the mic signal. The reverse stream is fed in by the SDK's
  //     ADM render side. BoardLoopback bypasses ADM playback entirely
  //     (custom AudioStream callback → ALSA writer thread), so APM never
  //     gets a reverse stream and AEC silently mis-fires: it treats every
  //     captured signal as undefined interference and attenuates voice
  //     instead of canceling echo. Real fix is wiring playback back through
  //     ADM, which is a larger refactor than Phase 7.2 — punted for now.
  //
  //   NS: on at kVeryHigh. Capture-side only, no reverse stream needed.
  //     Default kModerate is too gentle to be audible against ambient board
  //     fan / desk noise; kVeryHigh gives a clearly perceptible reduction.
  //
  //   HPF: on. Cuts < 80 Hz rumble (room HVAC, fan harmonics). Cheap and
  //     uncontroversial for voice.
  //
  //   Env escape hatch: BOARD_LOOPBACK_APM_OFF=1 disables NS+HPF too,
  //     reverting to Phase 6.3 fully-passthrough capture for A/B testing.
  webrtc::AudioProcessing::Config apm_config;
  const char *apm_off = std::getenv("BOARD_LOOPBACK_APM_OFF");
  bool full_off = apm_off && std::string(apm_off) == "1";
  apm_config.gain_controller1.enabled = false;
  apm_config.gain_controller2.enabled = false;
  apm_config.echo_canceller.enabled = false;
  apm_config.noise_suppression.enabled = !full_off;
  apm_config.noise_suppression.level =
      webrtc::AudioProcessing::Config::NoiseSuppression::kVeryHigh;
  apm_config.high_pass_filter.enabled = !full_off;
  RTC_LOG(LS_INFO) << "[apm] cfg: AEC=0 NS=" << !full_off
                   << "(VeryHigh) HPF=" << !full_off << " AGC1=0 AGC2=0";
  dependencies.audio_processing_builder =
      std::make_unique<webrtc::BuiltinAudioProcessingBuilder>(apm_config);

  webrtc::EnableMedia(dependencies);
  peer_factory_ =
      webrtc::CreateModularPeerConnectionFactory(std::move(dependencies));

  if (peer_factory_.get() == nullptr) {
    RTC_LOG_ERR(LS_ERROR) << "Failed to create PeerConnectionFactory";
    return;
  }
}

PeerConnectionFactory::~PeerConnectionFactory() {
  RTC_LOG(LS_VERBOSE) << "PeerConnectionFactory::~PeerConnectionFactory()";

  peer_factory_ = nullptr;
  rtc_runtime_->worker_thread()->BlockingCall(
      [this] { audio_device_ = nullptr; });
}

std::shared_ptr<PeerConnection> PeerConnectionFactory::create_peer_connection(
    RtcConfiguration config,
    rust::Box<PeerConnectionObserverWrapper> observer) const {
  std::shared_ptr<PeerConnection> pc = std::make_shared<PeerConnection>(
      rtc_runtime_, peer_factory_, std::move(observer));

  if (!pc->Initialize(to_native_rtc_configuration(config))) {
    throw std::runtime_error(serialize_error(to_error(webrtc::RTCError(
        webrtc::RTCErrorType::INTERNAL_ERROR, "failed to initialize pc"))));
  }

  return pc;
}

std::shared_ptr<VideoTrack> PeerConnectionFactory::create_video_track(
    rust::String label,
    std::shared_ptr<VideoTrackSource> source) const {
  return std::static_pointer_cast<VideoTrack>(
      rtc_runtime_->get_or_create_media_stream_track(
          peer_factory_->CreateVideoTrack(source->get(), label.c_str())));
}

std::shared_ptr<AudioTrack> PeerConnectionFactory::create_audio_track(
    rust::String label,
    std::shared_ptr<AudioTrackSource> source) const {
  return std::static_pointer_cast<AudioTrack>(
      rtc_runtime_->get_or_create_media_stream_track(
          peer_factory_->CreateAudioTrack(label.c_str(), source->get().get())));
}

RtpCapabilities PeerConnectionFactory::rtp_sender_capabilities(
    MediaType type) const {
  return to_rust_rtp_capabilities(peer_factory_->GetRtpSenderCapabilities(
      static_cast<webrtc::MediaType>(type)));
}

RtpCapabilities PeerConnectionFactory::rtp_receiver_capabilities(
    MediaType type) const {
  return to_rust_rtp_capabilities(peer_factory_->GetRtpReceiverCapabilities(
      static_cast<webrtc::MediaType>(type)));
}

std::shared_ptr<PeerConnectionFactory> create_peer_connection_factory() {
  return std::make_shared<PeerConnectionFactory>(RtcRuntime::create());
}

}  // namespace livekit_ffi
