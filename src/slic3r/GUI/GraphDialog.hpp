///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef _GRAPH_DIALOG_H_
#define _GRAPH_DIALOG_H_

#include <optional>

#include <wx/checkbox.h>
#include <wx/msgdlg.h>
#include <wx/spinctrl.h>
#include <wx/stattext.h>
#include <wx/textctrl.h>

#include "libslic3r/Config/ConfigOption.hpp" // for GraphSettings

#include "RammingChart.hpp"
#include "Widgets/SpinInput.hpp"
namespace Slic3r { namespace GUI {

class GraphPanel : public wxPanel
{
public:
    GraphPanel(wxWindow *parent, GraphData data,const GraphSettings &settings);
    GraphData get_data();
    bool      is_disabled();

private:
    Chart *           m_chart        = nullptr;
    SpinInputDouble *m_widget_min_x = nullptr;
    SpinInputDouble *m_widget_max_x = nullptr;
    SpinInputDouble *m_widget_min_y = nullptr;
    SpinInputDouble *m_widget_max_y = nullptr;
    SpinInputDouble *m_widget_x     = nullptr;
    SpinInputDouble *m_widget_y     = nullptr;
    double            m_last_min_x   = 0.f;
    double            m_last_max_x   = 1.f;
    double            m_last_min_y   = 0.f;
    double            m_last_max_y   = 1.f;

    // A reset target may select no point, one point, or only part of a list.
    // Those cases cannot always be encoded by the chart's non-empty X range,
    // so keep the exact indices while its abscissas and X range stay intact.
    // Ordinates do not determine which points belong to the active range.
    std::optional<std::pair<size_t, size_t>> m_reset_active_range;
    std::vector<float>                       m_reset_abscissas;
    double                                  m_reset_min_x = 0.0;
    double                                  m_reset_max_x = 0.0;
};

class GraphDialog : public wxDialog
{
public:
    GraphDialog(wxWindow *parent, const GraphData &parameters, const GraphSettings &settings);
    GraphData get_data() { return m_output_data; }
    bool      is_disabled() { return m_disabled; }

private:
    GraphPanel *m_panel_graph = nullptr;
    GraphData m_output_data;
    bool m_disabled = false;
};

}}     // namespace Slic3r::GUI
#endif  // _GRAPH_DIALOG_H_
