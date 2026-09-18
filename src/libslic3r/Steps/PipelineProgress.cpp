///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PipelineProgress.hpp"

#include <algorithm>
#include <stdexcept>

namespace Slic3r::Steps {

void PipelineProgress::initialize(
    const std::map<slicing_step_t, std::vector<std::string>> &units_by_step)
{
    this->reset();
    for (const std::pair<const slicing_step_t, std::vector<std::string>> &entry : units_by_step) {
        if (entry.second.empty())
            continue;
        Step &progress = m_steps[entry.first];
        progress.unit_ids = entry.second;
        m_total += entry.second.size();
    }
    m_active = m_total != 0;
}

void PipelineProgress::begin_step(const slicing_step_t step, const std::string &message)
{
    if (!m_active)
        return;
    if (m_steps.find(step) == m_steps.end())
        throw std::logic_error("A pipeline step started without a progress-plan entry.");

    m_status.set(m_percent, message, PrintStatus::SlicingStatus::MAIN_STATE);
    // Local progress belongs to the next plugin only. Clear the previous
    // detail before that plugin has a chance to publish its first update.
    m_status.set(-1, "", PrintStatus::SlicingStatus::SECONDARY_STATE);
    m_status.counter_reset();
}

void PipelineProgress::complete_plugin(const slicing_step_t step,
                                       const std::string &plugin_id)
{
    if (!m_active)
        return;
    const std::map<slicing_step_t, Step>::iterator found = m_steps.find(step);
    if (found == m_steps.end())
        throw std::logic_error("A plugin completed outside the pipeline progress plan.");

    Step &progress = found->second;
    if (progress.completed >= progress.unit_ids.size() ||
        progress.unit_ids[progress.completed] != plugin_id)
        throw std::logic_error("A pipeline plugin completed out of its planned order.");

    ++progress.completed;
    ++m_completed;
    const int percent = int((100 * m_completed) / m_total);
    m_percent = std::max(m_percent, percent);
    m_status.set_percent(m_percent, PrintStatus::Channel::Main);
    m_status.set(-1, "", PrintStatus::SlicingStatus::SECONDARY_STATE);
    m_status.counter_reset();
}

void PipelineProgress::complete_step(const slicing_step_t step)
{
    if (!m_active)
        return;
    const std::map<slicing_step_t, Step>::iterator found = m_steps.find(step);
    if (found == m_steps.end())
        throw std::logic_error("A completed step was absent from the pipeline progress plan.");

    // Some selected plugins legitimately have no payload for this print. Their
    // successful no-op invocation is accounted for at the step boundary.
    while (found->second.completed < found->second.unit_ids.size()) {
        const std::string plugin_id = found->second.unit_ids[found->second.completed];
        this->complete_plugin(step, plugin_id);
    }
}

void PipelineProgress::verify_step_complete(const slicing_step_t step) const
{
    if (!m_active)
        return;
    const std::map<slicing_step_t, Step>::const_iterator found = m_steps.find(step);
    if (found == m_steps.end() || found->second.completed != found->second.unit_ids.size())
        throw std::logic_error("A pipeline step returned before all planned units completed.");
}

void PipelineProgress::reset()
{
    m_steps.clear();
    m_total = 0;
    m_completed = 0;
    m_percent = 0;
    m_active = false;
}

} // namespace Slic3r::Steps
