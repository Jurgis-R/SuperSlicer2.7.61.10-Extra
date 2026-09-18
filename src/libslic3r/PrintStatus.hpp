///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_PrintStatus_hpp_
#define slic3r_PrintStatus_hpp_

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ObjectID.hpp"

namespace Slic3r {

class PrintBase;
class PrintObjectBase;

/*
PrintStatus is the thread-safe delivery channel between slicing producers and
their consumers. It deliberately does not decide what a percentage means: the
pipeline and legacy algorithms calculate progress, then publish it here.

Direct consumers such as the console receive every update synchronously. GUI
consumers receive a payload-free wakeup and later take an owned snapshot. Main
and secondary progress are coalesced independently, while control events keep
their insertion order because each one has an observable side effect.
*/
class PrintStatus
{
public:
    struct SlicingStatus {
        SlicingStatus() = default;
        SlicingStatus(int percent, const std::string &text, unsigned int flags = 0)
            : percent(percent), main_text(text), flags(flags) {}
        SlicingStatus(int percent,
                      const std::string &text,
                      const std::vector<std::string> &args,
                      unsigned int flags = 0)
            : percent(percent), main_text(text), args(args), flags(flags) {}
        SlicingStatus(const PrintBase &print, int warning_step);
        SlicingStatus(const PrintObjectBase &print_object, int warning_step);

        int                      percent { -1 };
        std::string              main_text;
        std::vector<std::string> args;

        enum FlagBits : uint16_t {
            DEFAULT                           = 0,
            RELOAD_SCENE                      = 1 << 1,
            RELOAD_SLA_SUPPORT_POINTS         = 1 << 2,
            RELOAD_SLA_PREVIEW                = 1 << 3,
            UPDATE_PRINT_STEP_WARNINGS        = 1 << 4,
            UPDATE_PRINT_OBJECT_STEP_WARNINGS = 1 << 5,
            SLICING_ENDED                     = 1 << 6,
            GCODE_ENDED                       = 1 << 7,
            MAIN_STATE                        = 1 << 8,
            SECONDARY_STATE                   = 1 << 9,
            FORCE_SHOW                        = 1 << 10
        };

        unsigned int flags { DEFAULT };
        ObjectID     warning_object_id;
        int          warning_step { -1 };
    };

    struct PendingBatch {
        uint64_t                     generation { 0 };
        std::optional<SlicingStatus> main;
        std::optional<SlicingStatus> secondary;
        std::vector<SlicingStatus>   control_events;

        bool empty() const
        {
            return !main.has_value() && !secondary.has_value() && control_events.empty();
        }
    };

    enum class Channel : uint8_t {
        Main,
        Secondary
    };

    using callback_type = std::function<void(const SlicingStatus &)>;
    using wakeup_callback_type = std::function<void()>;

    // Select one delivery mode. Changing mode also starts a fresh generation.
    void set_default();
    void set_silent();
    void set_callback(callback_type callback);
    void set_gui_wakeup_callback(wakeup_callback_type callback);

    // Discard queued data from an older run and return the new generation.
    uint64_t reset_pending();
    uint64_t generation() const { return m_generation.load(std::memory_order_acquire); }

    // Read changed progress channels and, when allowed, queued control events.
    PendingBatch take_pending(uint64_t generation, bool include_control_events);
    void request_pending_delivery() const;

    void set(int percent,
             const std::string &message,
             unsigned int flags = SlicingStatus::DEFAULT) const
    {
        this->set(percent, message, {}, flags);
    }
    void set(int percent,
             const std::string &message,
             const std::vector<std::string> &args,
             unsigned int flags = SlicingStatus::DEFAULT) const;

    // The hot percentage-only path does not lock channel text or arguments.
    void set_percent(int percent, Channel channel) const;

    // Legacy algorithms share these atomics to aggregate parallel local work.
    void counter_reset();
    void counter_add_max(int32_t increment) { m_counter_max += increment; }
    int32_t counter_max() const { return m_counter_max.load(std::memory_order_acquire); }
    int32_t counter_increment(int32_t increment = 1) { return m_counter.fetch_add(increment); }

    // Publish an already constructed status. The optional fallback preserves
    // warning output whose console text differs from its GUI control payload.
    void publish(SlicingStatus status,
                 const std::optional<std::string> &console_fallback = std::nullopt) const;

private:
    enum DirtyChannel : uint8_t {
        MAIN_DIRTY      = 1 << 0,
        SECONDARY_DIRTY = 1 << 1
    };

    void request_gui_wakeup() const;

    mutable std::mutex                   m_mutex;
    callback_type                        m_callback;
    mutable wakeup_callback_type         m_gui_wakeup_callback;
    mutable std::optional<SlicingStatus> m_main;
    mutable std::optional<SlicingStatus> m_secondary;
    mutable std::deque<SlicingStatus>    m_controls;
    mutable std::atomic_int32_t          m_main_percent { -1 };
    mutable std::atomic_int32_t          m_secondary_percent { -1 };
    mutable std::atomic<uint8_t>         m_dirty { 0 };
    mutable std::atomic_bool             m_event_waiting { false };
    mutable std::atomic_bool             m_gui_mode { false };
    mutable std::atomic_uint64_t         m_generation { 0 };
    std::atomic_int32_t                  m_counter { 0 };
    std::atomic_int32_t                  m_counter_max { 0 };
};

} // namespace Slic3r

#endif // slic3r_PrintStatus_hpp_
