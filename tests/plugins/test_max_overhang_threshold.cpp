#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Config/PrintConfig.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Steps/StepLayerHeightGeneration.hpp"
#include "libslic3r/Steps/StepPostSlicing.hpp"
#include "libslic3r/Steps/StepSlicing.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cmath>
#include <initializer_list>
#include <string>
#include <utility>
#include <vector>

/*
Tests for the max_overhang_threshold post-slicing plugin.

The fixture is a "T": a 4 mm wide stem carrying a 20 mm wide plate, with open
cantilevers on both sides of the stem. The cantilevers are anchored on one
side only, so the bridge detector finds no bridge over them and the plugin
must cut them back to the maximum slope.
*/

namespace {

using namespace Slic3r;

const char *const STANDARD_LAYER_HEIGHT_GENERATOR = "standard_layer_height_generator";
const char *const SLICE_VOLUME = "slice_volume";
const char *const BRIDGE_DETECTOR = "bridge_detector.default";
const char *const MAX_OVERHANG_THRESHOLD = "max_overhang_threshold";

// Restrict the active plugins to the ones a test needs, and restore the
// previous activation set at scope exit.
class ScopedActivePlugins
{
public:
    explicit ScopedActivePlugins(std::initializer_list<const char *> plugin_ids) :
        m_orchestrator(Orchestrator::instance())
    {
        m_previous_active_plugins.reserve(m_orchestrator.active_plugins().size());
        for (Plugin *plugin : m_orchestrator.active_plugins())
            m_previous_active_plugins.push_back(plugin);

        m_orchestrator.clear_active_plugins();
        for (const char *plugin_id : plugin_ids) {
            INFO("Activating test plugin " << plugin_id);
            REQUIRE(m_orchestrator.set_plugin_active(plugin_id, true));
        }
    }

    ~ScopedActivePlugins()
    {
        m_orchestrator.clear_active_plugins();
        for (Plugin *plugin : m_previous_active_plugins)
            m_orchestrator.set_plugin_active(plugin, true);
    }

private:
    Orchestrator &m_orchestrator;
    std::vector<Plugin *> m_previous_active_plugins;
};

TriangleMesh make_box(const double min_x, const double min_y, const double min_z,
                      const double max_x, const double max_y, const double max_z)
{
    TriangleMesh box(its_make_cube(max_x - min_x, max_y - min_y, max_z - min_z));
    box.translate(float(min_x), float(min_y), float(min_z));
    return box;
}

// Stem X=-2..2 from the bed to Z=10, plate X=-10..10 from Z=8.1 to Z=10,
// both Y=-10..10.
TriangleMesh make_t_shape()
{
    TriangleMesh mesh = make_box(-2., -10., 0., 2., 10., 10.);
    mesh.merge(make_box(-10., -10., 8.1, 10., 10., 10.));
    return mesh;
}

DynamicPrintConfig max_slope_config(const std::string &max_slope, const std::string &upper_layers)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"nozzle_diameter", "0.4"},
        {"overhangs_max_slope", max_slope},
        {"overhangs_bridge_upper_layers", upper_layers}
    });
    return config;
}

// Slice a T and run STEP_POST_SLICING with the max-slope plugin only.
void run_until_post_slicing(Model &model, Print &print, const DynamicPrintConfig &config)
{
    ModelObject *object = model.add_object();
    object->name = "max_overhang_threshold_test.stl";
    object->add_volume(make_t_shape(), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    model.center_instances_around_point({100, 100});
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.validate();
    print.set_status_silent();

    ScopedActivePlugins active({STANDARD_LAYER_HEIGHT_GENERATOR, SLICE_VOLUME, BRIDGE_DETECTOR, MAX_OVERHANG_THRESHOLD});
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
}

double area_mm2(const ExPolygons &areas)
{
    double out = 0.;
    for (const ExPolygon &area : areas)
        out += std::abs(area.area());
    return unscaled(unscaled(out));
}

// Area of the layer above that hangs more than slope_mm over `layer`.
double over_slope_area_mm2(const Layer &layer, const Layer &upper, const double slope_mm)
{
    return area_mm2(diff_ex(upper.lslices(), offset_ex(layer.lslices(), float(scale_d(slope_mm + 0.01)))));
}

} // namespace

// Regression: with the upper-layer bridge look-ahead enabled, a layer whose
// overhang has no bridge used to append a null Clipper operand and assert.
TEST_CASE("Max overhang threshold cuts unbridged cantilevers with the upper-layer look-ahead",
          "[plugins][post-slicing][max_overhang_threshold]")
{
    const std::string upper_layers = GENERATE(std::string("2"), std::string("0"), std::string("!2"));
    INFO("overhangs_bridge_upper_layers = " << upper_layers);
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    run_until_post_slicing(model, print, max_slope_config("1", upper_layers));
    const PrintObject &object = print.object(0);

    // The plate is cut so that each layer hangs at most 1 mm over the one below.
    for (size_t layer_idx = 0; layer_idx + 1 < object.layer_count(); ++layer_idx) {
        INFO("layer " << layer_idx);
        CHECK(over_slope_area_mm2(object.layer(layer_idx), object.layer(layer_idx + 1), 1.) < 0.05);
    }
    // The first plate layer keeps the 4 mm stem plus 1 mm on each side, over the
    // 20 mm depth, instead of the full 20 x 20 mm plate.
    size_t plate_idx = 0;
    while (plate_idx < object.layer_count() && object.layer(plate_idx).slice_z < 8.1)
        ++plate_idx;
    REQUIRE(plate_idx < object.layer_count());
    CHECK(area_mm2(object.layer(plate_idx).lslices()) == Approx(120.).margin(1.));
}
