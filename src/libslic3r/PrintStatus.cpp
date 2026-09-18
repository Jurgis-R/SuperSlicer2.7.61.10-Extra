///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PrintStatus.hpp"

#include <cstdio>
#include <iterator>
#include <utility>

#include "PrintBase.hpp"

namespace Slic3r {

namespace {

// Progress messages may replace an older value in the same GUI channel. Every
// other flag describes a one-shot action whose order must be preserved.
bool is_coalescable_progress(const PrintStatus::SlicingStatus &status)
{
    const unsigned int control_mask =
        PrintStatus::SlicingStatus::RELOAD_SCENE |
        PrintStatus::SlicingStatus::RELOAD_SLA_SUPPORT_POINTS |
        PrintStatus::SlicingStatus::RELOAD_SLA_PREVIEW |
        PrintStatus::SlicingStatus::UPDATE_PRINT_STEP_WARNINGS |
        PrintStatus::SlicingStatus::UPDATE_PRINT_OBJECT_STEP_WARNINGS |
        PrintStatus::SlicingStatus::SLICING_ENDED |
        PrintStatus::SlicingStatus::GCODE_ENDED |
        PrintStatus::SlicingStatus::FORCE_SHOW;
    return (status.flags & control_mask) == 0;
}

// Preserve the command-line presentation used before GUI coalescing was
// introduced. GUI mode stores untranslated text for the UI thread instead.
void print_default_status(const PrintStatus::SlicingStatus &status)
{
    printf("%d => ", status.percent);
    if (status.args.empty())
        printf("%s", status.main_text.c_str());
    else if (status.args.size() == 1)
        printf(status.main_text.c_str(), status.args.front().c_str());
    else if (status.args.size() == 2)
        printf(status.main_text.c_str(), status.args.front().c_str(), status.args.back().c_str());
    printf("\n");
}

} // namespace

PrintStatus::SlicingStatus::SlicingStatus(const PrintBase &print, const int warning_step)
    : flags(UPDATE_PRINT_STEP_WARNINGS), warning_object_id(print.id()), warning_step(warning_step)
{
}

PrintStatus::SlicingStatus::SlicingStatus(const PrintObjectBase &print_object,
                                          const int warning_step)
    : flags(UPDATE_PRINT_OBJECT_STEP_WARNINGS),
      warning_object_id(print_object.id()),
      warning_step(warning_step)
{
}

void PrintStatus::set_default()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_callback = nullptr;
        m_gui_wakeup_callback = nullptr;
        m_gui_mode.store(false, std::memory_order_release);
    }
    this->reset_pending();
}

void PrintStatus::set_silent()
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_callback = [](const SlicingStatus &) {};
        m_gui_wakeup_callback = nullptr;
        m_gui_mode.store(false, std::memory_order_release);
    }
    this->reset_pending();
}

void PrintStatus::set_callback(callback_type callback)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_callback = std::move(callback);
        m_gui_wakeup_callback = nullptr;
        m_gui_mode.store(false, std::memory_order_release);
    }
    this->reset_pending();
}

void PrintStatus::set_gui_wakeup_callback(wakeup_callback_type callback)
{
    const bool enabled = bool(callback);
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_callback = nullptr;
        m_gui_wakeup_callback = std::move(callback);
        m_gui_mode.store(enabled, std::memory_order_release);
    }
    this->reset_pending();
}

uint64_t PrintStatus::reset_pending()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    // Empty snapshots let producers update only a percentage before assigning
    // a label, without adding a lock to the hot path.
    m_main = SlicingStatus(-1, "", SlicingStatus::MAIN_STATE);
    m_secondary = SlicingStatus(-1, "", SlicingStatus::SECONDARY_STATE);
    m_controls.clear();
    m_main_percent.store(-1, std::memory_order_relaxed);
    m_secondary_percent.store(-1, std::memory_order_relaxed);
    m_dirty.store(0, std::memory_order_relaxed);
    m_event_waiting.store(false, std::memory_order_release);
    return m_generation.fetch_add(1, std::memory_order_acq_rel) + 1;
}

PrintStatus::PendingBatch PrintStatus::take_pending(const uint64_t generation,
                                                    const bool include_control_events)
{
    PendingBatch batch;
    batch.generation = generation;
    if (generation != m_generation.load(std::memory_order_acquire))
        return batch;

    // Clear before taking the snapshot. A producer racing with this read will
    // schedule another wakeup instead of losing its update.
    m_event_waiting.store(false, std::memory_order_release);

    std::lock_guard<std::mutex> lock(m_mutex);
    if (generation != m_generation.load(std::memory_order_relaxed))
        return batch;

    const uint8_t dirty = m_dirty.exchange(0, std::memory_order_acq_rel);
    if ((dirty & MAIN_DIRTY) != 0 && m_main.has_value()) {
        batch.main = m_main;
        batch.main->percent = m_main_percent.load(std::memory_order_acquire);
    }
    if ((dirty & SECONDARY_DIRTY) != 0 && m_secondary.has_value()) {
        batch.secondary = m_secondary;
        batch.secondary->percent = m_secondary_percent.load(std::memory_order_acquire);
    }
    if (include_control_events) {
        batch.control_events.assign(
            std::make_move_iterator(m_controls.begin()),
            std::make_move_iterator(m_controls.end()));
        m_controls.clear();
    }
    return batch;
}

void PrintStatus::request_pending_delivery() const
{
    bool has_pending = m_dirty.load(std::memory_order_acquire) != 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        has_pending = has_pending || !m_controls.empty();
    }
    if (has_pending)
        this->request_gui_wakeup();
}

void PrintStatus::request_gui_wakeup() const
{
    if (m_event_waiting.exchange(true, std::memory_order_acq_rel))
        return;

    wakeup_callback_type callback;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        callback = m_gui_wakeup_callback;
    }
    if (!callback) {
        m_event_waiting.store(false, std::memory_order_release);
        return;
    }
    try {
        callback();
    } catch (...) {
        // A failed scheduler must not permanently suppress later GUI updates.
        m_event_waiting.store(false, std::memory_order_release);
        throw;
    }
}

void PrintStatus::publish(SlicingStatus status,
                          const std::optional<std::string> &console_fallback) const
{
    if ((status.flags & (SlicingStatus::MAIN_STATE | SlicingStatus::SECONDARY_STATE)) == 0)
        status.flags |= SlicingStatus::MAIN_STATE;

    callback_type callback;
    bool gui_mode = false;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        callback = m_callback;
        gui_mode = bool(m_gui_wakeup_callback);
        if (gui_mode) {
            if (!is_coalescable_progress(status)) {
                m_controls.push_back(std::move(status));
            } else {
                uint8_t dirty = 0;
                if ((status.flags & SlicingStatus::MAIN_STATE) != 0) {
                    m_main_percent.store(status.percent, std::memory_order_release);
                    m_main = status;
                    m_main->flags &= ~SlicingStatus::SECONDARY_STATE;
                    m_main->flags |= SlicingStatus::MAIN_STATE;
                    dirty |= MAIN_DIRTY;
                }
                if ((status.flags & SlicingStatus::SECONDARY_STATE) != 0) {
                    m_secondary_percent.store(status.percent, std::memory_order_release);
                    m_secondary = std::move(status);
                    m_secondary->flags &= ~SlicingStatus::MAIN_STATE;
                    m_secondary->flags |= SlicingStatus::SECONDARY_STATE;
                    dirty |= SECONDARY_DIRTY;
                }
                m_dirty.fetch_or(dirty, std::memory_order_release);
            }
        }
    }

    if (gui_mode) {
        this->request_gui_wakeup();
    } else if (callback) {
        callback(status);
    } else if (console_fallback.has_value()) {
        if (!console_fallback->empty())
            printf("%s\n", console_fallback->c_str());
    } else {
        print_default_status(status);
    }
}

void PrintStatus::set(const int percent,
                      const std::string &message,
                      const std::vector<std::string> &args,
                      const unsigned int flags) const
{
    this->publish(SlicingStatus(percent, message, args, flags));
}

void PrintStatus::set_percent(const int percent, const Channel channel) const
{
    const bool secondary = channel == Channel::Secondary;
    const uint8_t dirty = secondary ? SECONDARY_DIRTY : MAIN_DIRTY;
    if (!m_gui_mode.load(std::memory_order_acquire)) {
        this->set(percent,
                  "",
                  secondary ? SlicingStatus::SECONDARY_STATE : SlicingStatus::MAIN_STATE);
        return;
    }

    std::atomic_int32_t &stored_percent = secondary ? m_secondary_percent : m_main_percent;
    stored_percent.store(percent, std::memory_order_release);
    m_dirty.fetch_or(dirty, std::memory_order_release);
    this->request_gui_wakeup();
}

void PrintStatus::counter_reset()
{
    m_counter.store(0, std::memory_order_release);
    m_counter_max.store(0, std::memory_order_release);
}

} // namespace Slic3r
