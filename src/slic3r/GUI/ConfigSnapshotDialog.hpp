///|/ Copyright (c) SuperSlicer 2026 Durand R?mi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2019 Oleksandra Iushchenko @YuSanka, Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

#ifndef slic3r_GUI_ConfigSnapshotDialog_hpp_
#define slic3r_GUI_ConfigSnapshotDialog_hpp_

#include <functional>
#include <string>

#include <wx/html/htmlwin.h>
#include <wx/intl.h>
#include <wx/wx.h>

#include "GUI.hpp"
#include "GUI_Utils.hpp"
namespace Slic3r {
namespace GUI {

namespace Config {
	class SnapshotDB;
}

class ConfigSnapshotDialog : public DPIDialog
{
public:
    using CreateSnapshotCallback = std::function<std::string(wxWindow *)>;
    // Prepare the application for replacing its presets. Returning false keeps
    // this modal manager open and leaves the selected snapshot unchanged.
    using PrepareActivationCallback = std::function<bool()>;

    ConfigSnapshotDialog(const Config::SnapshotDB &snapshot_db, const wxString &id,
                         CreateSnapshotCallback create_snapshot_callback,
                         PrepareActivationCallback prepare_activation_callback);
    const std::string& snapshot_to_activate() const { return m_snapshot_to_activate; }

protected:
    void on_dpi_changed(const wxRect &suggested_rect) override;

private:
    // Rebuild the HTML after creating a snapshot so the dialog may remain open
    // while displaying the new active entry.
    void refresh_snapshot_list();
    void onCreateSnapshot(wxEvent &);
    void onLinkClicked(wxHtmlLinkEvent &event);
    void onCloseDialog(wxEvent &);

    const Config::SnapshotDB &m_snapshot_db;
    std::string               m_active_snapshot;
    CreateSnapshotCallback    m_create_snapshot_callback;
    PrepareActivationCallback m_prepare_activation_callback;

    // If set, it contains a snapshot ID to be restored after the dialog closes.
    std::string m_snapshot_to_activate;

    wxHtmlWindow *html { nullptr };
};

} // namespace GUI
} // namespace Slic3r

#endif /* slic3r_GUI_ConfigSnapshotDialog_hpp_ */
