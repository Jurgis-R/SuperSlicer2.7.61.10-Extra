///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/

// This updater is the plugin counterpart to PresetUpdater. It reuses the
// repository description, tag and archive protocol from libslic3r. Its
// callbacks expose transport-neutral results so GUI code can localize them.

#ifndef slic3r_Updater_PluginUpdater_hpp_
#define slic3r_Updater_PluginUpdater_hpp_

#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "libslic3r/Plugins/PluginRepository.hpp"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Updater/RepositoryUpdater.hpp"
#include "libslic3r/Updater/UpdaterError.hpp"

namespace Slic3r {

enum class PluginChangelogSource { None, Package, Repository };

struct PluginAvailable : public RepositoryPackageVersion {
    // NotChecked until a local version.ini has actually been read.
    PluginPackageMetadata metadata;
    // Non-empty for structurally valid cached packages, including incompatible
    // ones. ABI compatibility must still be checked before scheduling.
    std::string local_directory;
    std::string notes;
    PluginChangelogSource notes_source = PluginChangelogSource::None;
    // Optional-file errors are displayed independently of ABI/install errors.
    std::string notes_error;
    // Refresh local notes without replacing repository notes when no entry exists.
    void load_package_changelog();
};

struct PluginSync {
    RepositoryDescription description;
    // Present for a system package whose version is dictated by the current
    // application bundle rather than by an updater selection.
    std::optional<PluginInstalledVersion> bundled_exact_version;
    // Desired selection from activated.ini; it may not be active until restart.
    PluginInstalledVersion installed_version;
    PluginPackageMetadata installed_metadata;
    bool is_installed = false;
    // Actual live-directory version, independent of pending install/removal.
    // Missing or unreadable metadata leaves this absent, never presumed active.
    std::optional<PluginInstalledVersion> live_version;
    // Compare package and build identities without inspecting files from the GUI.
    bool installation_pending() const;
    bool is_bundled_exact() const { return bundled_exact_version.has_value(); }
    // True when the repository cache contains this plugin description. The
    // entry may exist before any package version has been downloaded.
    bool has_cache = false;
    // sync_error is populated only for Failed. Starting a new attempt clears
    // it before exposing InProgress to callers and dialog copies.
    RepositorySyncState sync_state = RepositorySyncState::Unchecked;
    UpdaterError sync_error;
    // Startup package diagnostics are copied from Orchestrator when this model
    // is rebuilt. They describe executable loading, independently from the
    // repository synchronization state above.
    std::optional<PluginPackageLoadReport> load_report;
    bool can_upgrade = false;
    std::vector<PluginAvailable> available_packages;

    UpdaterError parse_tags(const std::string &json);
    void sort_available();

    // Return the first verified ABI-compatible entry from the sorted list.
    // The pointer refers to this snapshot rather than to updater-owned data.
    const PluginAvailable *best_available() const;
};

// Background preparation may fail for one repository while updates from
// other repositories remain usable. Keeping those failures by package lets
// the GUI report partial work without treating the whole scan as unusable.
struct PluginUpdatePreparationResult {
    size_t available_updates = 0;
    UpdaterError operation_error;
    std::map<std::string, UpdaterError> package_errors;
};

// A bulk update is best-effort. Successfully scheduled packages remain
// selected even when another package fails later in the same operation.
struct PluginUpdateBatchResult {
    std::vector<std::string> scheduled_packages;
    UpdaterError operation_error;
    std::map<std::string, UpdaterError> package_errors;
};

// Return true when the current compatible offers contain a package that was
// not dismissed, or a package version newer than the dismissed one.
bool has_new_plugin_package_updates(
    const std::map<std::string, std::string> &available,
    const std::map<std::string, std::string> &dismissed);

class PluginUpdater : public RepositoryUpdater {
public:
    PluginUpdater() = default;

    // Uses an injected transport instead of the process HTTP backend. Pending
    // requests retain their callbacks, so the transport must outlive them.
    explicit PluginUpdater(UpdaterHttpTransport &http_transport) : RepositoryUpdater(http_transport) {}
    PluginUpdater(const PluginUpdater &) = delete;
    PluginUpdater(PluginUpdater &&) = delete;
    PluginUpdater &operator=(const PluginUpdater &) = delete;
    PluginUpdater &operator=(PluginUpdater &&) = delete;
    ~PluginUpdater() override;

    // Reload local descriptions and selected package versions. This does not
    // contact the network; sync_async() performs that work later.
    void reload_all_plugins();
    void sync_async(std::function<void(int)> callback_result, bool force = false);

    // Force-refresh repository tags and cache the newest ABI-compatible update
    // for each installed package. With network disabled this only inspects the
    // already validated cache and completes synchronously.
    void prepare_updates(bool allow_network,
                         std::function<void(PluginUpdatePreparationResult)> callback_result);

    // Schedule every currently compatible update for the next application
    // start. The callback receives all successes and per-package failures.
    void install_all_updates(std::function<void(PluginUpdateBatchResult)> callback_result);

    // Prefer embedded notes for each version, then use the repository. Notes cached less
    // than 24 hours ago are reused unless force is true. The callback reports
    // whether every version succeeded; partial results remain available.
    void load_changelogs(const std::string &plugin_id,
                             std::function<void(bool)> callback_result,
                             bool force = false);
    void download_new_repo(const std::string &rest_url, std::function<void(UpdaterError)> callback_result);

    // Import an unpacked package directory on the serialized worker and reload
    // the detached model after publication. Generic description.ini and
    // version.ini files are generated when metadata can be derived from the
    // payload or defaults. The callback runs on an updater/HTTP worker; GUI
    // callers must marshal it to their owner thread.
    void cache_plugin_directory(const boost::filesystem::path &package_directory,
                                std::function<void(UpdaterError)> callback_result);
    void install_plugin(const std::string &plugin_id,
                        const PluginAvailable &version,
                        std::function<void(UpdaterError)> callback_result);
    // Record a package removal for the next startup without unloading the DLL
    // currently used by this process.
    void uninstall_plugin(const std::string &plugin_id, std::function<void(UpdaterError)> callback_result);
    void clear_cache_plugin(const std::string &plugin_id, std::function<void(UpdaterError)> callback_result);

    size_t count_available() const;
    size_t count_updates() const;
    // Snapshot the exact package versions represented by count_updates().
    // Pending installs and application-managed packages are excluded.
    std::map<std::string, std::string> available_update_versions() const;

    // These accessors return detached copies. A caller may keep or move them
    // while HTTP workers publish newer data without invalidating the copy.
    std::vector<std::string> plugin_ids() const;
    std::vector<PluginSync> plugins() const;
    std::optional<PluginSync> plugin(const std::string &id) const;

private:
    struct UpdatePreparationState;

    // Acquire the shared repository mutation gate and build a once-only
    // terminal callback which releases it before notifying the caller.
    bool start_plugin_change(const char *context,
                             std::function<void(UpdaterError)> callback_result,
                             std::function<void(UpdaterError)> &complete);
    UpdaterError cache_plugin_directory_files(const boost::filesystem::path &package_directory);
    void schedule_cached_plugin_install_async(const std::string &plugin_id,
                                              const PluginAvailable &version,
                                              std::function<void(UpdaterError)> callback_result);
    void begin_update_preparation(
        std::function<void(PluginUpdatePreparationResult)> callback_result);
    void prepare_next_update_candidate(
        std::shared_ptr<UpdatePreparationState> state);
    void cache_update_candidate(
        const std::string &plugin_id,
        const PluginAvailable &version,
        std::function<void(UpdaterError)> callback_result);
    void publish_cached_update_candidate(
        const std::string &plugin_id,
        const PluginAvailable &version);
    void update_plugin(const std::string &plugin_id, bool force);
    PluginSync *find_plugin_unlocked(const std::string &id);
    const PluginSync *find_plugin_unlocked(const std::string &id) const;
    UpdaterError schedule_cached_plugin_install(const std::string &plugin_id, const PluginAvailable &version);
    UpdaterError uninstall_plugin_files(const std::string &plugin_id);
    UpdaterError clear_cache_plugin_files(const std::string &plugin_id);
    int update_count() override;

    std::map<std::string, PluginSync> m_plugins;
};

} // namespace Slic3r

#endif // slic3r_Updater_PluginUpdater_hpp_
