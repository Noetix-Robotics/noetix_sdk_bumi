#include "MediaController.h"
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace noetix;

PYBIND11_MODULE(mediacontrol_py, m) {

        // ── media::Header ──────────────────────────────────
        py::class_<media::Header>(m, "MediaHeader")
            .def(py::init<>())
            .def_readwrite("message_id", &media::Header::message_id)
            .def_readwrite("timestamp_us", &media::Header::timestamp_us)
            .def_readwrite("sn", &media::Header::sn);

        // ── media::WorkStatus ──────────────────────────────
        py::enum_<media::WorkStatus>(m, "WorkStatus")
            .value("READY", media::WorkStatus::READY)
            .value("SLEEPED", media::WorkStatus::SLEEPED)
            .value("WAKEUPED", media::WorkStatus::WAKEUPED)
            .value("EXIT", media::WorkStatus::EXIT)
            .export_values();

        // ── media::SystemControlType ──────────────────────────────
        py::enum_<media::SystemControlType>(m, "SystemControlType")
            .value("TO_WAKEUP", media::SystemControlType::TO_WAKEUP)
            .value("TO_SLEEP", media::SystemControlType::TO_SLEEP)
            .value("TO_RESET", media::SystemControlType::TO_RESET)
            .export_values();

        // ── media::StatusChangeReason ──────────────────────
        py::enum_<media::StatusChangeReason>(m, "StatusChangeReason")
            .value("SYSTEM_LAUNCH", media::StatusChangeReason::SYSTEM_LAUNCH)
            .value("CMD_RESET", media::StatusChangeReason::CMD_RESET)
            .value("AUDIO_WAKEUPED", media::StatusChangeReason::AUDIO_WAKEUPED)
            .value("CMD_WAKEUPED", media::StatusChangeReason::CMD_WAKEUPED)
            .value("AUDIO_SLEEPED", media::StatusChangeReason::AUDIO_SLEEPED)
            .value("CMD_SLEEPED", media::StatusChangeReason::CMD_SLEEPED)
            .value("TIMEOUT_SLEEPED",
                   media::StatusChangeReason::TIMEOUT_SLEEPED)
            .value("ERROR_SLEEPED", media::StatusChangeReason::ERROR_SLEEPED)
            .export_values();

        // ── media::SystemStatus ────────────────────────────
        py::class_<media::SystemStatus>(m, "SystemStatus")
            .def(py::init<>())
            .def_readwrite("header", &media::SystemStatus::header)
            .def_readwrite("value", &media::SystemStatus::value)
            .def_readwrite("reason", &media::SystemStatus::reason);

        // ── media::SystemError ─────────────────────────────
        py::class_<media::SystemError>(m, "SystemError")
            .def(py::init<>())
            .def_readwrite("header", &media::SystemError::header)
            .def_readwrite("code", &media::SystemError::code)
            .def_readwrite("message", &media::SystemError::message);

        // ── media::AudioStream ─────────────────────────────
        py::class_<media::AudioStream>(m, "AudioStream")
            .def(py::init<>())
            .def_readwrite("header", &media::AudioStream::header)
            .def_readwrite("timestamp_us", &media::AudioStream::timestamp_us)
            .def_readwrite("channels", &media::AudioStream::channels)
            .def_readwrite("sample_rate", &media::AudioStream::sample_rate)
            .def_readwrite("format", &media::AudioStream::format)
            .def_readwrite("duration_ms", &media::AudioStream::duration_ms)
            .def_readwrite("audio_data", &media::AudioStream::audio_data);

        // ── media::VideoStream ─────────────────────────────
        py::class_<media::VideoStream>(m, "VideoStream")
            .def(py::init<>())
            .def_readwrite("header", &media::VideoStream::header)
            .def_readwrite("timestamp_us", &media::VideoStream::timestamp_us)
            .def_readwrite("format", &media::VideoStream::format)
            .def_readwrite("width", &media::VideoStream::width)
            .def_readwrite("height", &media::VideoStream::height)
            .def_readwrite("fps", &media::VideoStream::fps)
            .def_readwrite("video_data", &media::VideoStream::video_data);

        // ── MediaController ────────────────────────────────
        py::class_<MediaController>(m, "MediaController")
            .def_static("instance", &MediaController::Instance,
                        py::return_value_policy::reference)

            .def("init", &MediaController::init)

            // System Control
            .def("set_system_control", &MediaController::set_system_control)

            // System State
            .def("subscribe_system_status",
                 &MediaController::subscribe_system_status, py::arg("callback"))
            .def("subscribe_system_error",
                 &MediaController::subscribe_system_error, py::arg("callback"))

            // Volume
            .def("get_volume", &MediaController::get_volume)
            .def("set_volume", &MediaController::set_volume, py::arg("value"))

            // Common Config
            .def("get_timeout", &MediaController::get_timeout)
            .def("set_timeout", &MediaController::set_timeout,
                 py::arg("timeout_ms"))
            .def("get_audio_cue_enable", &MediaController::get_audio_cue_enable)
            .def("set_audio_cue_enable", &MediaController::set_audio_cue_enable,
                 py::arg("enable"))
            .def("get_internal_capture_audio_data_to_agent_enable",
                 &MediaController::
                     get_internal_capture_audio_data_to_agent_enable)
            .def("set_internal_capture_audio_data_to_agent_enable",
                 &MediaController::
                     set_internal_capture_audio_data_to_agent_enable,
                 py::arg("enable"))
            .def("get_external_custom_audio_data_to_agent_enable",
                 &MediaController::
                     get_external_custom_audio_data_to_agent_enable)
            .def("set_external_custom_audio_data_to_agent_enable",
                 &MediaController::
                     set_external_custom_audio_data_to_agent_enable,
                 py::arg("enable"))
            .def("get_internal_agent_audio_data_to_playback_enable",
                 &MediaController::
                     get_internal_agent_audio_data_to_playback_enable)
            .def("set_internal_agent_audio_data_to_playback_enable",
                 &MediaController::
                     set_internal_agent_audio_data_to_playback_enable,
                 py::arg("enable"))
            .def("get_external_custom_audio_data_to_playback_enable",
                 &MediaController::
                     get_external_custom_audio_data_to_playback_enable)
            .def("set_external_custom_audio_data_to_playback_enable",
                 &MediaController::
                     set_external_custom_audio_data_to_playback_enable,
                 py::arg("enable"))
            .def("get_internal_capture_video_data_to_agent_enable",
                 &MediaController::
                     get_internal_capture_video_data_to_agent_enable)
            .def("set_internal_capture_video_data_to_agent_enable",
                 &MediaController::
                     set_internal_capture_video_data_to_agent_enable,
                 py::arg("enable"))
            .def("get_external_custom_video_data_to_agent_enable",
                 &MediaController::
                     get_external_custom_video_data_to_agent_enable)
            .def("set_external_custom_video_data_to_agent_enable",
                 &MediaController::
                     set_external_custom_video_data_to_agent_enable,
                 py::arg("enable"))
            .def("get_external_custom_audio_data_to_agent_use_internal_3a",
                 &MediaController::
                     get_external_custom_audio_data_to_agent_use_internal_3a)
            .def("set_external_custom_audio_data_to_agent_use_internal_3a",
                 &MediaController::
                     set_external_custom_audio_data_to_agent_use_internal_3a,
                 py::arg("enable"))

            // Wakeup Config
            .def("get_wakeup_response_words",
                 &MediaController::get_wakeup_response_words)
            .def("set_wakeup_response_words",
                 &MediaController::set_wakeup_response_words, py::arg("words"))
            .def("get_sleep_response_words",
                 &MediaController::get_sleep_response_words)
            .def("set_sleep_response_words",
                 &MediaController::set_sleep_response_words, py::arg("words"))
            .def("get_wakeup_words", &MediaController::get_wakeup_words)

            // Audio Stream
            .def("subscribe_internal_audio_capture",
                 &MediaController::subscribe_internal_audio_capture,
                 py::arg("callback"))
            .def("subscribe_internal_audio_playback",
                 &MediaController::subscribe_internal_audio_playback,
                 py::arg("callback"))

            // Video Stream
            .def("subscribe_internal_video_capture",
                 &MediaController::subscribe_internal_video_capture,
                 py::arg("callback"))
            .def("subscribe_internal_video_desensed",
                 &MediaController::subscribe_internal_video_desensed,
                 py::arg("callback"))

            // Video Control
            .def("pause_video_capture", &MediaController::pause_video_capture)
            .def("resume_video_capture", &MediaController::resume_video_capture)

            // External Stream
            .def("publish_external_video_stream",
                 &MediaController::publish_external_video_stream,
                 py::arg("stream"))
            .def("publish_external_audio_stream",
                 &MediaController::publish_external_audio_stream,
                 py::arg("stream"))
            .def("publish_external_audio_playback_stream",
                 &MediaController::publish_external_audio_playback_stream,
                 py::arg("stream"))

            // Audio Control
            .def("pause_audio_capture", &MediaController::pause_audio_capture)
            .def("resume_audio_capture", &MediaController::resume_audio_capture)
            .def("pause_audio_playback", &MediaController::pause_audio_playback)
            .def("resume_audio_playback",
                 &MediaController::resume_audio_playback);
}
