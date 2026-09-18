///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/ Copyright (c) Prusa Research 2018 - 2023 Oleksandra Iushchenko @YuSanka, Pavel Mikuš @Godrak, Vojtěch Bubník @bubnikv, Roman Beránek @zavorka, Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include "PrintBase.hpp"

#include <iterator>
#include <regex>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include "Exception.hpp"
#include "I18N.hpp"

namespace Slic3r
{

void PrintTryCancel::operator()() const
{
    m_print->throw_if_canceled();
}

size_t PrintStateBase::g_last_timestamp = 0;

namespace {

// Progress messages may replace an older value in the same GUI channel. Every
// other flag describes a one-shot action whose order must be preserved.
bool is_coalescable_progress(const PrintBase::SlicingStatus &status)
{
    const unsigned int control_mask =
        PrintBase::SlicingStatus::RELOAD_SCENE |
        PrintBase::SlicingStatus::RELOAD_SLA_SUPPORT_POINTS |
        PrintBase::SlicingStatus::RELOAD_SLA_PREVIEW |
        PrintBase::SlicingStatus::UPDATE_PRINT_STEP_WARNINGS |
        PrintBase::SlicingStatus::UPDATE_PRINT_OBJECT_STEP_WARNINGS |
        PrintBase::SlicingStatus::SLICING_ENDED |
        PrintBase::SlicingStatus::GCODE_ENDED |
        PrintBase::SlicingStatus::FORCE_SHOW;
    return (status.flags & control_mask) == 0;
}

// Preserve the command-line presentation used before GUI coalescing was
// introduced. GUI mode never reaches this formatter because it stores the
// untranslated message and arguments for the UI thread.
void print_default_status(const PrintBase::SlicingStatus &status)
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

void PrintBase::set_status_default()
{
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        m_status_callback = nullptr;
        m_gui_status_wakeup_callback = nullptr;
        m_gui_status_mode.store(false, std::memory_order_release);
    }
    this->reset_pending_gui_statuses();
}

void PrintBase::set_status_silent()
{
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        m_status_callback = [](const SlicingStatus &) {};
        m_gui_status_wakeup_callback = nullptr;
        m_gui_status_mode.store(false, std::memory_order_release);
    }
    this->reset_pending_gui_statuses();
}

void PrintBase::set_status_callback(status_callback_type callback)
{
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        m_status_callback = std::move(callback);
        m_gui_status_wakeup_callback = nullptr;
        m_gui_status_mode.store(false, std::memory_order_release);
    }
    this->reset_pending_gui_statuses();
}

void PrintBase::set_gui_status_wakeup_callback(status_wakeup_callback_type callback)
{
    const bool enabled = bool(callback);
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        m_status_callback = nullptr;
        m_gui_status_wakeup_callback = std::move(callback);
        m_gui_status_mode.store(enabled, std::memory_order_release);
    }
    this->reset_pending_gui_statuses();
}

uint64_t PrintBase::reset_pending_gui_statuses()
{
    std::lock_guard<std::mutex> lock(m_gui_status_mutex);
    // Keep an empty snapshot for each channel so a producer may publish only a
    // new percentage without first allocating or locking a label update.
    m_gui_main_status = SlicingStatus(-1, "", SlicingStatus::MAIN_STATE);
    m_gui_secondary_status = SlicingStatus(-1, "", SlicingStatus::SECONDARY_STATE);
    m_gui_control_statuses.clear();
    m_gui_main_percent.store(-1, std::memory_order_relaxed);
    m_gui_secondary_percent.store(-1, std::memory_order_relaxed);
    m_gui_status_dirty.store(0, std::memory_order_relaxed);
    m_gui_event_waiting.store(false, std::memory_order_release);
    return m_gui_status_generation.fetch_add(1, std::memory_order_acq_rel) + 1;
}

PrintBase::PendingStatusBatch PrintBase::take_pending_gui_statuses(
    const uint64_t generation,
    const bool include_control_events)
{
    PendingStatusBatch batch;
    batch.generation = generation;
    if (generation != m_gui_status_generation.load(std::memory_order_acquire))
        return batch;

    // Clear the wakeup before taking the snapshot. A producer racing with this
    // read will therefore enqueue a second wakeup instead of losing its update.
    m_gui_event_waiting.store(false, std::memory_order_release);

    std::lock_guard<std::mutex> lock(m_gui_status_mutex);
    if (generation != m_gui_status_generation.load(std::memory_order_relaxed))
        return batch;

    const uint8_t dirty = m_gui_status_dirty.exchange(0, std::memory_order_acq_rel);
    if ((dirty & GUI_STATUS_MAIN_DIRTY) != 0 && m_gui_main_status.has_value()) {
        batch.main = m_gui_main_status;
        batch.main->percent = m_gui_main_percent.load(std::memory_order_acquire);
    }
    if ((dirty & GUI_STATUS_SECONDARY_DIRTY) != 0 && m_gui_secondary_status.has_value()) {
        batch.secondary = m_gui_secondary_status;
        batch.secondary->percent = m_gui_secondary_percent.load(std::memory_order_acquire);
    }
    if (include_control_events) {
        batch.control_events.assign(
            std::make_move_iterator(m_gui_control_statuses.begin()),
            std::make_move_iterator(m_gui_control_statuses.end()));
        m_gui_control_statuses.clear();
    }
    return batch;
}

void PrintBase::request_pending_gui_status_delivery() const
{
    bool has_pending = m_gui_status_dirty.load(std::memory_order_acquire) != 0;
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        has_pending = has_pending || !m_gui_control_statuses.empty();
    }
    if (has_pending)
        this->request_gui_status_wakeup();
}

void PrintBase::request_gui_status_wakeup() const
{
    if (m_gui_event_waiting.exchange(true, std::memory_order_acq_rel))
        return;

    status_wakeup_callback_type callback;
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        callback = m_gui_status_wakeup_callback;
    }
    if (!callback) {
        m_gui_event_waiting.store(false, std::memory_order_release);
        return;
    }

    try {
        callback();
    } catch (...) {
        // A failed scheduler must not permanently suppress later GUI updates.
        m_gui_event_waiting.store(false, std::memory_order_release);
        throw;
    }
}

void PrintBase::publish_status(SlicingStatus status,
                               const std::optional<std::string> &console_fallback) const
{
    if ((status.flags & (SlicingStatus::MAIN_STATE | SlicingStatus::SECONDARY_STATE)) == 0)
        status.flags |= SlicingStatus::MAIN_STATE;

    status_callback_type callback;
    bool gui_mode = false;
    {
        std::lock_guard<std::mutex> lock(m_gui_status_mutex);
        callback = m_status_callback;
        gui_mode = bool(m_gui_status_wakeup_callback);
        if (gui_mode) {
            if (!is_coalescable_progress(status)) {
                m_gui_control_statuses.push_back(std::move(status));
            } else {
                uint8_t dirty = 0;
                if ((status.flags & SlicingStatus::MAIN_STATE) != 0) {
                    m_gui_main_percent.store(status.percent, std::memory_order_release);
                    m_gui_main_status = status;
                    m_gui_main_status->flags &= ~SlicingStatus::SECONDARY_STATE;
                    m_gui_main_status->flags |= SlicingStatus::MAIN_STATE;
                    dirty |= GUI_STATUS_MAIN_DIRTY;
                }
                if ((status.flags & SlicingStatus::SECONDARY_STATE) != 0) {
                    m_gui_secondary_percent.store(status.percent, std::memory_order_release);
                    m_gui_secondary_status = std::move(status);
                    m_gui_secondary_status->flags &= ~SlicingStatus::MAIN_STATE;
                    m_gui_secondary_status->flags |= SlicingStatus::SECONDARY_STATE;
                    dirty |= GUI_STATUS_SECONDARY_DIRTY;
                }
                m_gui_status_dirty.fetch_or(dirty, std::memory_order_release);
            }
        }
    }

    if (gui_mode) {
        this->request_gui_status_wakeup();
    } else if (callback) {
        callback(status);
    } else if (console_fallback.has_value()) {
        if (!console_fallback->empty())
            printf("%s\n", console_fallback->c_str());
    } else {
        print_default_status(status);
    }
}

void PrintBase::set_status(const int percent,
                           const std::string &message,
                           const std::vector<std::string> &args,
                           const unsigned int flags) const
{
    this->publish_status(SlicingStatus(percent, message, args, flags));
}

void PrintBase::set_status_percent(const int percent, const bool secondary) const
{
    const uint8_t dirty = secondary ? GUI_STATUS_SECONDARY_DIRTY : GUI_STATUS_MAIN_DIRTY;
    if (!m_gui_status_mode.load(std::memory_order_acquire)) {
        this->set_status(percent, "", secondary ? SlicingStatus::SECONDARY_STATE : SlicingStatus::MAIN_STATE);
        return;
    }

    std::atomic_int32_t &stored_percent = secondary ? m_gui_secondary_percent : m_gui_main_percent;
    stored_percent.store(percent, std::memory_order_release);
    m_gui_status_dirty.fetch_or(dirty, std::memory_order_release);
    this->request_gui_status_wakeup();
}

// Update "scale", "input_filename", "input_filename_base" placeholders from the current m_objects.
void PrintBase::update_object_placeholders(DynamicConfig &config, const std::string & /* default_output_ext */) const
{
    // get the first input file name
    std::string input_file;
    std::vector<std::string> v_scale;
    int num_objects = 0;
    int num_instances = 0;
	for (const ModelObject &model_object : m_model.objects()) {
		ModelInstance *printable = nullptr;
		for (ModelInstance *model_instance : model_object.instances)
			if (model_instance->is_printable()) {
				printable = model_instance;
				++ num_instances;
			}
		if (printable) {
            ++ num_objects;
	        // CHECK_ME -> Is the following correct ?
			v_scale.push_back("x:" + boost::lexical_cast<std::string>(printable->get_scaling_factor(X) * 100) +
				"% y:" + boost::lexical_cast<std::string>(printable->get_scaling_factor(Y) * 100) +
				"% z:" + boost::lexical_cast<std::string>(printable->get_scaling_factor(Z) * 100) + "%");
	        if (input_file.empty())
	            input_file = model_object.name.empty() ? model_object.input_file : model_object.name;
	    }
    }
    
    config.set_key_value("num_objects", new ConfigOptionInt(num_objects));
    config.set_key_value("num_instances", new ConfigOptionInt(num_instances));

    config.set_key_value("scale", new ConfigOptionStrings(v_scale));
    if (! input_file.empty()) {
        // get basename with and without suffix
        const std::string input_filename = boost::filesystem::path(input_file).filename().string();
        const std::string input_filename_base = input_filename.substr(0, input_filename.find_last_of("."));
//        config.set_key_value("input_filename", new ConfigOptionString(input_filename_base + default_output_ext));
        config.set_key_value("input_filename_base", new ConfigOptionString(input_filename_base));
    }
}

// Generate an output file name based on the format template, default extension, and template parameters
// (timestamps, object placeholders derived from the model, current placeholder prameters, print statistics - config_override)
std::string PrintBase::output_filename(const std::string &format, const std::string &default_ext, const std::string &filename_base, const DynamicConfig *config_override) const
{
    DynamicConfig cfg;
    if (config_override != nullptr)
    	cfg = *config_override;
    cfg.set_key_value("version", new ConfigOptionString(std::string(SLIC3R_VERSION)));
    PlaceholderParser::update_timestamp(cfg);
    this->update_object_placeholders(cfg, default_ext);
    if (! filename_base.empty()) {
//		cfg.set_key_value("input_filename", new ConfigOptionString(filename_base + default_ext));
		cfg.set_key_value("input_filename_base", new ConfigOptionString(filename_base));
    }
    try {
        const int64_t raw_initial_extruder = config_override != nullptr &&
                config_override->option("initial_extruder") != nullptr &&
                config_override->option("initial_extruder")->type() == coInt ?
            config_override->option("initial_extruder")->get_int() : 0;
        if (raw_initial_extruder < EXTRUDER_ID_UNKNOWN ||
            raw_initial_extruder > (std::numeric_limits<extruder_index_t>::max)())
            throw std::invalid_argument("initial_extruder does not fit in extruder_index_t");
        const extruder_index_t extruder_initial = static_cast<extruder_index_t>(raw_initial_extruder);
        boost::filesystem::path filepath = format.empty() ?
            cfg.opt_string("input_filename_base") + default_ext :
            this->placeholder_parser().process(format, extruder_initial, &cfg);
        //remove unwanted characters
        std::string forbidden_base;
        if (const ConfigOptionString* opt = this->placeholder_parser().external_config()->option<ConfigOptionString>("gcode_filename_illegal_char")) {
            forbidden_base = opt->value;
        }
        if (!forbidden_base.empty()) {
            const std::string filename_init = filepath.stem().string();
            std::string filename = filename_init;
            std::string extension = filepath.extension().string();
            //remove {print_time} and things like that that may be computed after, and re-put them inside it after the replace.
            std::regex placehoder = std::regex("\\{[a-z_]+\\}");
            std::smatch matches;
            std::string::const_iterator searchStart(filename_init.cbegin());
            while (std::regex_search(searchStart, filename_init.cend(), matches, placehoder)) {
                for (size_t i = 0; i < matches.size(); i++) {
                    filename.replace(matches.position(i), matches.length(i), matches.length(i), '_');
                }
                searchStart = matches.suffix().first;
            }
            //remove unwanted characters from the cleaned string
            bool regexp_used = false;
            if (forbidden_base.front() == '(' || forbidden_base.front() == '[') {
                try {
                    filename = std::regex_replace(filename, std::regex(forbidden_base), "_");
                    regexp_used = true;
                }catch(std::exception){}
            }
            if (!regexp_used) {
                for(size_t i = 0; i < forbidden_base.size(); i++)
                    std::replace(filename.begin(), filename.end(), forbidden_base.at(i), '_');
            }
            //re-put {print_time} and things like that
            searchStart = (filename_init.cbegin());
            while (std::regex_search(searchStart, filename_init.cend(), matches, placehoder)) {
                for (size_t i = 0; i < matches.size(); i++) {
                    filename.replace(matches.position(i), matches.length(i), matches.str());
                }
                searchStart = matches.suffix().first;
            }
            // set the path var
            filepath = filename + extension;
        }
        if (filepath.extension().empty())
            filepath.replace_extension(default_ext);
        return filepath.string();
    } catch (std::runtime_error &err) {
        throw Slic3r::PlaceholderParserError(_u8L("Failed processing of the output_filename_format template.") + "\n" + err.what());
    }
}

std::string PrintBase::output_filepath(const std::string &path, const std::string &filename_base) const
{
    // if we were supplied no path, generate an automatic one based on our first object's input file
    if (path.empty())
        // get the first input file name
        return (boost::filesystem::path(m_model.propose_export_file_name_and_path()).parent_path() / this->output_filename(filename_base)).make_preferred().string();
    
    // if we were supplied a directory, use it and append our automatically generated filename
    boost::filesystem::path p(path);
    if (boost::filesystem::is_directory(p))
        return (p / this->output_filename(filename_base)).make_preferred().string();
    
    // if we were supplied a file which is not a directory, use it
    return path;
}

void PrintBase::status_update_warnings(int step, PrintStateBase::WarningLevel /* warning_level */, const std::string &message, const PrintObjectBase* print_object)
{
    SlicingStatus status = print_object ? SlicingStatus(*print_object, step) : SlicingStatus(*this, step);
    const std::string console_message = message.empty() ? std::string() :
        std::string(print_object != nullptr ? "print_object" : "print") +
            " warning: " + message;
    this->publish_status(std::move(status), console_message);
}

std::mutex& PrintObjectBase::state_mutex(PrintBase *print)
{ 
	return print->state_mutex();
}

std::function<void()> PrintObjectBase::cancel_callback(PrintBase *print)
{ 
	return print->cancel_callback();
}

void PrintObjectBase::status_update_warnings(PrintBase *print, int step, PrintStateBase::WarningLevel warning_level, const std::string &message)
{
    print->status_update_warnings(step, warning_level, message, this);
}

} // namespace Slic3r
