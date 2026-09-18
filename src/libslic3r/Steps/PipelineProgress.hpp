///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Steps_PipelineProgress_hpp_
#define slic3r_Steps_PipelineProgress_hpp_

#include <map>
#include <string>
#include <vector>

#include "libslic3r/PrintStatus.hpp"
#include "libslic3r/PrintSteps.hpp"

namespace Slic3r::Steps {

/*
PipelineProgress translates successful top-level pipeline units into the main
status percentage. It owns the execution accounting but delegates all delivery
and GUI synchronization to PrintStatus.

Unit order is fixed by initialize(). A non-empty unit id names a selected
plugin; an empty id represents host-only work. Units complete only after their
work returns successfully, so failures and cancellation cannot advance the
reported percentage.
*/
class PipelineProgress
{
public:
    explicit PipelineProgress(PrintStatus &status) : m_status(status) {}

    void initialize(const std::map<slicing_step_t, std::vector<std::string>> &units_by_step);
    bool active() const { return m_active; }
    int percent() const { return m_percent; }

    void begin_step(slicing_step_t step, const std::string &message);
    void complete_plugin(slicing_step_t step, const std::string &plugin_id);
    void complete_step(slicing_step_t step);
    void verify_step_complete(slicing_step_t step) const;
    void reset();

private:
    struct Step {
        std::vector<std::string> unit_ids;
        size_t                   completed { 0 };
    };

    PrintStatus                         &m_status;
    std::map<slicing_step_t, Step>       m_steps;
    size_t                               m_total { 0 };
    size_t                               m_completed { 0 };
    int                                  m_percent { 0 };
    bool                                 m_active { false };
};

} // namespace Slic3r::Steps

#endif // slic3r_Steps_PipelineProgress_hpp_
