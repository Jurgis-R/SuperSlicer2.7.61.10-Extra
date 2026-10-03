#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Api/host/Plugin.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
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
Tests for the overhang_slope_fill post-slicing plugin.

The fixtures are built from axis-aligned boxes:
- a "T": a 4 mm wide stem carrying a 20 mm wide plate, open cantilevers on
  both sides of the stem;
- an "arch": two pillars carrying a slab over a 12 mm gap, i.e. a bridge.
Only the layer-height, slicing, bridge-detector and fill plugins are active,
so the observed layer geometry is the direct output of the fill. The fill
works per layer, so layers are addressed by their distance to the first
plate/slab layer rather than by Z.
*/

namespace {

using namespace Slic3r;

const char *const STANDARD_LAYER_HEIGHT_GENERATOR = "standard_layer_height_generator";
const char *const SLICE_VOLUME = "slice_volume";
const char *const BRIDGE_DETECTOR = "bridge_detector.default";
const char *const OVERHANG_SLOPE_FILL = "overhang_slope_fill";

struct Box
{
    double min_x = 0.;
    double min_y = 0.;
    double min_z = 0.;
    double max_x = 0.;
    double max_y = 0.;
    double max_z = 0.;
};

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

TriangleMesh make_box(const Box &box)
{
    std::vector<Vec3f> vertices = {
        {float(box.min_x), float(box.min_y), float(box.min_z)},
        {float(box.max_x), float(box.min_y), float(box.min_z)},
        {float(box.max_x), float(box.max_y), float(box.min_z)},
        {float(box.min_x), float(box.max_y), float(box.min_z)},
        {float(box.min_x), float(box.min_y), float(box.max_z)},
        {float(box.max_x), float(box.min_y), float(box.max_z)},
        {float(box.max_x), float(box.max_y), float(box.max_z)},
        {float(box.min_x), float(box.max_y), float(box.max_z)}
    };
    std::vector<Vec3i32> faces = {
        {0, 2, 1}, {0, 3, 2},
        {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4},
        {1, 2, 6}, {1, 6, 5},
        {2, 3, 7}, {2, 7, 6},
        {3, 0, 4}, {3, 4, 7}
    };
    return TriangleMesh(std::move(vertices), std::move(faces));
}

TriangleMesh make_boxes(std::initializer_list<Box> boxes)
{
    TriangleMesh mesh;
    for (const Box &box : boxes)
        mesh.merge(make_box(box));
    return mesh;
}

// Z of the plate/slab bottom face. It is kept away from the slicing planes so
// the first plate layer does not depend on how a face lying exactly on a
// plane is sliced.
constexpr double k_plate_z = 8.1;

// Stem X=-2..2 from the bed to Z=10, plate X=-10..10 from k_plate_z to Z=10,
// both Y=-10..10. The stem section is 4 x 20 = 80 mm2.
TriangleMesh make_t_shape()
{
    return make_boxes({{-2., -10., 0., 2., 10., 10.}, {-10., -10., k_plate_z, 10., 10., 10.}});
}

// Expected layer area k layers below the plate with a 1 mm slope: the
// erosion shrinks the plate by k mm on every side, so the fill covers
// X=2..10-k on both sides of the stem and Y=-(10-k)..10-k.
double expected_t_area_mm2(const int k)
{
    const double reach = 10. - double(k);
    return reach <= 2. ? 80. : 80. + 2. * (reach - 2.) * 2. * reach;
}

// Pillars X=-10..-6 and X=6..10 up to k_plate_z, slab X=-10..10 from
// k_plate_z to Z=10.
TriangleMesh make_arch()
{
    return make_boxes({{-10., -4., 0., -6., 4., k_plate_z},
                       {6., -4., 0., 10., 4., k_plate_z},
                       {-10., -4., k_plate_z, 10., 4., 10.}});
}

DynamicPrintConfig fill_config(const std::string &slope, const std::string &bridge_threshold)
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"nozzle_diameter", "0.4"},
        {"overhangs_fill_slope", slope},
        {"overhangs_fill_bridge_threshold", bridge_threshold}
    });
    return config;
}

// One object made of `part`, plus optional modifier boxes carrying their own
// overhangs_fill_slope value.
Model make_model(TriangleMesh &&part, std::initializer_list<std::pair<Box, std::string>> slope_modifiers = {})
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "overhang_slope_fill_test.stl";
    ModelVolume *part_volume = object->add_volume(std::move(part), ModelVolumeType::MODEL_PART, false);
    part_volume->set_type(ModelVolumeType::MODEL_PART);
    for (const std::pair<Box, std::string> &modifier : slope_modifiers) {
        ModelVolume *volume = object->add_volume(make_box(modifier.first), ModelVolumeType::PARAMETER_MODIFIER, false);
        volume->set_type(ModelVolumeType::PARAMETER_MODIFIER);
        volume->config.set_deserialize_strict("overhangs_fill_slope", modifier.second);
    }
    object->add_instance();
    return model;
}

// Slice the model and run STEP_POST_SLICING with the fill plugin only.
void run_until_post_slicing(Model &model, Print &print, const DynamicPrintConfig &config)
{
    model.center_instances_around_point({100, 100});
    for (ModelObject &object : model.objects()) {
        object.ensure_on_bed();
        print.auto_assign_extruders(&object);
    }
    print.apply(model, config);
    print.validate();
    print.set_status_silent();

    ScopedActivePlugins active({STANDARD_LAYER_HEIGHT_GENERATOR, SLICE_VOLUME, BRIDGE_DETECTOR, OVERHANG_SLOPE_FILL});
    Orchestrator &orchestrator = Orchestrator::instance();
    Steps::StepLayerHeightGeneration::run_step(orchestrator, print);
    Steps::StepSlicing::run_step(orchestrator, print);
    Steps::StepPostSlicing::run_step(orchestrator, print);
}

// Index of the first object layer sliced above z_mm.
size_t first_layer_above(const PrintObject &object, const double z_mm)
{
    size_t layer_idx = 0;
    while (layer_idx < object.layer_count() && object.layer(layer_idx).slice_z < z_mm)
        ++layer_idx;
    REQUIRE(layer_idx < object.layer_count());
    return layer_idx;
}

// The layer `k` layers below the first layer of the plate (or slab).
const Layer &layer_below_plate(const PrintObject &object, const size_t k)
{
    const size_t plate_idx = first_layer_above(object, k_plate_z);
    REQUIRE(plate_idx >= k);
    return object.layer(plate_idx - k);
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

// Raw region slices must not overlap, so later steps see a clean partition.
void require_disjoint_regions(const Layer &layer)
{
    for (size_t region_idx = 0; region_idx < layer.region_count(); ++region_idx)
        for (size_t other_idx = region_idx + 1; other_idx < layer.region_count(); ++other_idx) {
            INFO("region overlap " << region_idx << " / " << other_idx);
            CHECK(area_mm2(intersection_ex(layer.region(region_idx).get_raw_slices(),
                                           layer.region(other_idx).get_raw_slices())) < 0.001);
        }
}

} // namespace

TEST_CASE("Overhang slope fill leaves slices unchanged when disabled", "[plugins][post-slicing][overhang_slope_fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model = make_model(make_t_shape());
    Print print;
    run_until_post_slicing(model, print, fill_config("0", "0"));

    // Below the plate, only the stem exists.
    const PrintObject &object = print.object(0);
    CHECK(area_mm2(layer_below_plate(object, 1).lslices()) == Approx(80.).margin(0.1));
    CHECK(area_mm2(object.layer(0).lslices()) == Approx(80.).margin(0.1));
}

TEST_CASE("Overhang slope fill supports a cantilever down to the stem", "[plugins][post-slicing][overhang_slope_fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model = make_model(make_t_shape());
    Print print;
    // Bridges are filled too, so the result does not depend on how the bridge
    // detector classifies the cantilevers.
    run_until_post_slicing(model, print, fill_config("1", "0"));
    const PrintObject &object = print.object(0);

    // Each layer may hang at most 1 mm over the layer below.
    for (size_t layer_idx = 0; layer_idx + 1 < object.layer_count(); ++layer_idx) {
        INFO("layer " << layer_idx);
        CHECK(over_slope_area_mm2(object.layer(layer_idx), object.layer(layer_idx + 1), 1.) < 0.05);
        require_disjoint_regions(object.layer(layer_idx));
    }

    // The fill is an inverted pyramid: one connected island under the plate,
    // shrinking by 1 mm per side and per layer until it reaches the stem
    // 8 layers below the plate.
    CHECK(layer_below_plate(object, 1).islands().size() == 1);
    for (size_t k = 1; k <= 9; ++k) {
        INFO(k << " layers below the plate");
        CHECK(area_mm2(layer_below_plate(object, k).lslices()) == Approx(expected_t_area_mm2(int(k))).margin(1.));
    }
}

TEST_CASE("Overhang slope fill keeps bridgeable spans open", "[plugins][post-slicing][overhang_slope_fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    SECTION("bridges are accepted: the gap under the slab stays empty")
    {
        Model model = make_model(make_arch());
        Print print;
        // A disabled bridge limit accepts every bridge length.
        run_until_post_slicing(model, print, fill_config("1", "!0"));
        const Layer &below_slab = layer_below_plate(print.object(0), 1);
        CHECK(below_slab.islands().size() == 2);
        CHECK(area_mm2(below_slab.lslices()) == Approx(64.).margin(1.));
    }

    SECTION("bridge limit 0: the gap is filled like any overhang")
    {
        Model model = make_model(make_arch());
        Print print;
        run_until_post_slicing(model, print, fill_config("1", "0"));
        const Layer &below_slab = layer_below_plate(print.object(0), 1);
        CHECK(below_slab.islands().size() == 1);
        // Pillars plus the slab eroded by 1 mm between them: 12 x 6 mm.
        CHECK(area_mm2(below_slab.lslices()) == Approx(64. + 12. * 6.).margin(1.));
    }
}

TEST_CASE("Overhang slope fill grows the unique neighbour region", "[plugins][post-slicing][overhang_slope_fill]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The object disables the fill. A modifier covering the whole width from
    // Z=6.1 up to the plate enables it, so the fill starts under the plate,
    // belongs to the modifier region, and stops where the modifier ends.
    const double modifier_bottom_z = 6.1;
    Model model = make_model(make_t_shape(), {{{-12., -12., modifier_bottom_z, 12., 12., k_plate_z}, "1"}});
    Print print;
    run_until_post_slicing(model, print, fill_config("0", "0"));
    const PrintObject &object = print.object(0);

    // Inside the modifier Z range the stem belongs to the modifier region, so
    // the whole layer, fill included, is in one region.
    const Layer &filled = layer_below_plate(object, 1);
    CHECK(area_mm2(filled.lslices()) == Approx(expected_t_area_mm2(1)).margin(1.));
    require_disjoint_regions(filled);
    std::vector<size_t> filled_regions;
    for (size_t region_idx = 0; region_idx < filled.region_count(); ++region_idx)
        if (!filled.region(region_idx).get_raw_slices().empty())
            filled_regions.push_back(region_idx);
    REQUIRE(filled_regions.size() == 1);
    CHECK(area_mm2(filled.region(filled_regions.front()).get_raw_slices()) ==
          Approx(expected_t_area_mm2(1)).margin(1.));

    // Below the modifier, the default region has a slope of 0: nothing is
    // added and only the stem remains, in another region.
    const Layer &below_modifier = object.layer(first_layer_above(object, modifier_bottom_z) - 1);
    CHECK(area_mm2(below_modifier.lslices()) == Approx(80.).margin(0.1));
    CHECK(below_modifier.region(filled_regions.front()).get_raw_slices().empty());
}