// Guard metadata ownership, read-only capabilities and checkpoint editing windows against regressions.
#include <catch2/catch.hpp>

#include "plugin_test_helpers.hpp"
#include "perimeter_test_helpers.hpp"
#include "test_data.hpp"

#include "libslic3r/Api/plugin/c/slic3r_data_tree.h"
#include "libslic3r/Api/plugin/cpp/DataTreeViews.hpp"
#include "libslic3r/Api/plugin/cpp/SurfaceViews.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionTreeVisitors.hpp"
#include "libslic3r/Api/plugin/cpp/ExtrusionViews.hpp"
#include "libslic3r/Api/plugin/c/slic3r_orchestrator.h"
#include "libslic3r/Api/host/Orchestrator.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PluginProperty.hpp"
#include "libslic3r/Plugins/PluginApiCompatibility.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"
#include "libslic3r/Surface.hpp"

#include <utility>
#include <functional>
#include <type_traits>
#include <stdexcept>
#include <vector>

namespace {
using namespace Slic3r;

struct TestSurfacePayload
{
    uint32_t priority = 0;
    uint32_t layer_count = 0;
};

/* Payload without a static id, used to exercise the runtime-key API. */
struct RuntimePropertyPayload
{
    uint32_t value = 0;
    uint32_t generation = 0;
};

/* Deliberately incompatible payload used to test namespaced layout checks. */
struct IncompatibleRuntimePropertyPayload
{
    uint8_t value = 0;
};

/* Reads one inherited runtime property from each visited extrusion leaf. */
class RuntimePropertyVisitor : public slic3r_api::ExtrusionTreeVisitor<>
{
public:
    explicit RuntimePropertyVisitor(
        const slic3r_api::PluginPropertyKey<RuntimePropertyPayload> &key) :
        m_key(key)
    {
    }

    uint32_t visited_value = 0;

protected:
    void visit_leaf(slic3r_api::MutableExtrusionEntity) override
    {
        if (const RuntimePropertyPayload *property = current_property(m_key))
            visited_value = property->value;
    }

private:
    slic3r_api::PluginPropertyKey<RuntimePropertyPayload> m_key;
};

orchestrator_handle *test_orchestrator_handle()
{
    return reinterpret_cast<orchestrator_handle *>(&Orchestrator::instance());
}

slic3r_api::PluginPropertyKey<TestSurfacePayload> test_surface_payload_key()
{
    return slic3r_api::PluginPropertyKey<TestSurfacePayload>::register_dynamic(
        test_orchestrator_handle(), "tests.surface.payload");
}

slic3r_api::PluginProperties property_view(PluginPropertyContainer &container)
{
    return slic3r_api::PluginProperties(
        reinterpret_cast<plugin_property_container_handle *>(&container));
}

ExPolygon rectangle_expolygon(double min_x, double min_y, double max_x, double max_y)
{
    return ExPolygon(Polygon({
        Point(scale_i(min_x), scale_i(min_y)),
        Point(scale_i(max_x), scale_i(min_y)),
        Point(scale_i(max_x), scale_i(max_y)),
        Point(scale_i(min_x), scale_i(max_y))
    }));
}

} // namespace

TEST_CASE("Data-tree property getters preserve exact inclusive editing windows",
          "[plugins][properties][freeze]")
{
    static_assert(std::is_same<decltype(object_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_island_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_island_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Test::PerimeterPluginTests::PreparedPerimeterPrint prepared;
    Test::PerimeterPluginTests::prepare_cube_print(prepared, Test::PerimeterPluginTests::perimeter_config({}));
    Print &print = prepared.print;
    PrintObject &object = print.object(0);
    Layer &main = object.layer(0);
    PrintObject &global = print.mutable_auxiliary_object();
    Layer *auxiliary[2];
    PrintObject *owners[2] = {&object, &global};
    for (size_t idx = 0; idx < 2; ++idx) {
        layer_handle *handle = object_add_auxiliary_layer(reinterpret_cast<const object_handle *>(owners[idx]),
                                                        scale_i(0.2), scale_i(0.2), scale_i(0.1));
        REQUIRE(handle != nullptr);
        auxiliary[idx] = reinterpret_cast<Layer *>(handle);
        *reinterpret_cast<ExPolygons *>(layer_region_borrow_mutable_slices(
            reinterpret_cast<layer_region_handle *>(&auxiliary[idx]->region(0)))) =
            ExPolygons{rectangle_expolygon(0., 0., 10., 10.)};
        layer_recompute_slices_and_islands_from_layer_regions(handle);
        layer_add_regions_to_islands(handle);
        REQUIRE(auxiliary[idx]->islands().size() == 1);
    }
    struct Node {
        slicing_step_t last;
        std::function<plugin_property_container_handle *()> write;
        std::function<const plugin_property_container_handle *()> read;
    };
    std::vector<Node> nodes;
    for (PrintObject *owner : owners) {
        object_handle *handle = reinterpret_cast<object_handle *>(owner);
        nodes.push_back({STEP_FINALIZE_SLICING, [handle]() { return object_get_properties_mutable(handle); },
                        [handle]() { return object_get_properties(handle); }});
    }
    for (Layer *layer : {&main, auxiliary[0], auxiliary[1]}) {
        const slicing_step_t last = layer == &main ? STEP_POST_SLICING : STEP_FINALIZE_SLICING;
        layer_handle *handle = reinterpret_cast<layer_handle *>(layer);
        layer_region_handle *region = reinterpret_cast<layer_region_handle *>(&layer->region(0));
        layer_island_handle *island = reinterpret_cast<layer_island_handle *>(&layer->island(0));
        LayerRegionIsland &bucket = layer->island(0).get_or_add_region_island(layer->island(0).regions(), 0);
        layer_region_island_handle *group = reinterpret_cast<layer_region_island_handle *>(&bucket);
        nodes.push_back({last, [handle]() { return layer_get_properties_mutable(handle); },
                        [handle]() { return layer_get_properties(handle); }});
        nodes.push_back({last, [region]() { return layer_region_get_properties_mutable(region); },
                        [region]() { return layer_region_get_properties(region); }});
        nodes.push_back({last, [island]() { return layer_island_get_properties_mutable(island); },
                        [island]() { return layer_island_get_properties(island); }});
        nodes.push_back({STEP_FINALIZE_SLICING, [group]() { return layer_region_island_get_properties_mutable(group); },
                        [group]() { return layer_region_island_get_properties(group); }});
    }
    for (const Node &node : nodes) {
        print.set_running_step(node.last);
        REQUIRE(print.running_step_may_edit(node.last));
        plugin_property_container_handle *properties = node.write();
        REQUIRE(properties != nullptr);
        c_layer_support_property *payload = static_cast<c_layer_support_property *>(
            plugin_property_get_or_add_data_mutable(test_orchestrator_handle(), properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
        REQUIRE(payload != nullptr);
        payload->interface_id = 73;
        CHECK(node.read() == properties);
        print.set_running_step(node.last == STEP_POST_SLICING ? STEP_PRE_PERIMETER : STEP_ORDERING);
        CHECK_FALSE(print.running_step_may_edit(node.last));
        REQUIRE(plugin_property_data(node.read(), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != nullptr);
        CHECK(static_cast<const c_layer_support_property *>(
            plugin_property_data(node.read(), PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT))->interface_id == 73);
    }
    print.set_running_step(STEP_NONE);
    CHECK(object_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_region_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_island_get_properties_mutable(nullptr) == nullptr);
    CHECK(layer_region_island_get_properties_mutable(nullptr) == nullptr);
}

TEST_CASE("Surface properties require a scratch editing capability regardless of pipeline phase",
          "[plugins][properties][surface][freeze]")
{
    static_assert(std::is_same<decltype(surface_get_properties(nullptr)), const plugin_property_container_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_island_get_fill_surfaces(nullptr)), const surface_collection_handle *>::value);
    static_assert(std::is_same<decltype(layer_region_island_get_fill_surface(nullptr, 0)), const surface_handle *>::value);
    static_assert(!std::is_constructible<slic3r_api::MutableSurface, surface_handle *>::value);
    static_assert(std::is_same<decltype(std::declval<slic3r_api::Surface>().properties()), slic3r_api::ConstPluginProperties>::value);
    Test::Plugins::ensure_plugin_test_runtime_initialized();
    Print print;
    PluginStorage storage;
    slic3r_api::StoredSurfaceCollection scratch(reinterpret_cast<storage_handle *>(&storage));
    ExPolygon geometry = rectangle_expolygon(0., 0., 10., 10.);
    scratch.append(slic3r_api::ExPolygon(reinterpret_cast<const expolygon_handle *>(&geometry)), RAW_SURFACE_TYPE_NONE);
    for (const slicing_step_t step : {STEP_FINALIZE_SLICING, STEP_ORDERING, STEP_GCODE}) {
        print.set_running_step(step);
        scratch.mutable_at(0).mutable_properties().get_or_add(slic3r_api::LayerSupportProperty::key).interface_id = uint32_t(step);
        REQUIRE(scratch.readonly().at(0).properties().get(slic3r_api::LayerSupportProperty::key) != nullptr);
        CHECK(scratch.readonly().at(0).properties().get(slic3r_api::LayerSupportProperty::key)->interface_id == uint32_t(step));
    }
    CHECK(surface_collection_get_properties_mutable(nullptr, 0) == nullptr);
    CHECK(surface_collection_get_properties_mutable(scratch.mutable_handle(), 1) == nullptr);
    print.set_running_step(STEP_NONE);
}

TEST_CASE("Property freeze rejects the previous data-tree ABI major",
          "[plugins][properties][abi]")
{
    static_assert(SLIC3R_PLUGIN_API_DATA_TREE_MAJOR == 4u);
    std::vector<PluginApiRequirement> requirements = {
        {"slic3r_plugin_types.h", {SLIC3R_PLUGIN_API_PLUGIN_TYPES_MAJOR, SLIC3R_PLUGIN_API_PLUGIN_TYPES_MINOR}},
        {"slic3r_data_tree.h", {3, 0}}
    };
    CHECK_FALSE(validate_plugin_api_requirements(requirements).compatible());
    requirements.back().version = {SLIC3R_PLUGIN_API_DATA_TREE_MAJOR, SLIC3R_PLUGIN_API_DATA_TREE_MINOR};
    CHECK(validate_plugin_api_requirements(requirements).compatible());
}

TEST_CASE("PluginPropertyKey gives built-in and dynamic properties one access model",
          "[plugins][properties][property-key]")
{
    using slic3r_api::LayerSupportProperty;
    using slic3r_api::EPropertyOverhang;
    using slic3r_api::PluginProperties;
    using slic3r_api::PluginPropertyKey;
    using slic3r_api::StoredExtrusionEntity;

    const PluginPropertyKey<RuntimePropertyPayload> first_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.first");
    const PluginPropertyKey<RuntimePropertyPayload> repeated_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.first");
    const PluginPropertyKey<RuntimePropertyPayload> second_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.second");

    CHECK(first_key.type() == repeated_key.type());
    CHECK(first_key.type() != second_key.type());
    CHECK(first_key.orchestrator() == test_orchestrator_handle());

    // Generic data-tree storage uses the key without separately passing an id
    // or orchestrator at the point where the payload is accessed.
    PluginPropertyContainer native_properties;
    PluginProperties properties(reinterpret_cast<plugin_property_container_handle *>(&native_properties));
    first_key.get_or_add(properties).value = 17;
    second_key.get_or_add(properties).value = 29;
    REQUIRE(properties.get(first_key) != nullptr);
    REQUIRE(first_key.get(properties) != nullptr);
    REQUIRE(second_key.get(properties) != nullptr);
    CHECK(first_key.get(properties)->value == 17);
    CHECK(second_key.get(properties)->value == 29);
    REQUIRE(first_key.get_mutable(properties) != nullptr);
    first_key.get_mutable(properties)->generation = 3;
    CHECK(first_key.get(properties)->generation == 3);
    CHECK(first_key.has(properties));

    // A built-in payload uses the same operations; only construction of its
    // key differs because its numeric identity is part of the public ABI.
    CHECK(LayerSupportProperty::key.type() == PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT);
    CHECK(LayerSupportProperty::key.orchestrator() == nullptr);
    properties.get_or_add(LayerSupportProperty::key).interface_id = 42;
    REQUIRE(properties.get(LayerSupportProperty::key) != nullptr);
    REQUIRE(LayerSupportProperty::key.get(properties) != nullptr);
    CHECK(LayerSupportProperty::key.get(properties)->interface_id == 42);

    // Extrusion storage consumes the exact same dynamic key and therefore
    // cannot accidentally use another orchestrator during payload creation.
    PluginStorage storage;
    StoredExtrusionEntity extrusion(reinterpret_cast<storage_handle *>(&storage));
    extrusion.get_or_add(first_key).value = 61;
    REQUIRE(extrusion.get(first_key) != nullptr);
    CHECK(first_key.has(extrusion));
    REQUIRE(first_key.get(extrusion) != nullptr);
    CHECK(first_key.get(extrusion)->value == 61);
    REQUIRE(first_key.get_mutable(extrusion) != nullptr);
    first_key.get_mutable(extrusion)->generation = 8;
    CHECK(first_key.get(extrusion)->generation == 8);
    CHECK(first_key.remove(extrusion));
    CHECK_FALSE(first_key.has(extrusion));

    CHECK(EPropertyOverhang::key.type() == EXTRUSION_PROPERTY_TYPE_OVERHANG);
    CHECK(EPropertyOverhang::key.orchestrator() == nullptr);
    extrusion.get_or_add(EPropertyOverhang::key).start_distance_from_prev_layer = 0.25f;
    REQUIRE(extrusion.get(EPropertyOverhang::key) != nullptr);
    REQUIRE(EPropertyOverhang::key.get(extrusion) != nullptr);
    CHECK(EPropertyOverhang::key.get(extrusion)->start_distance_from_prev_layer == Approx(0.25f));

    // Extrusion-axis operations use the same built-in property mechanism. The
    // semantic operation remains separate from geometry and may therefore be
    // attached to an empty Retract or Unretract event leaf.
    CHECK(slic3r_api::EPropertyExtrusionAxis::key.type() ==
          EXTRUSION_PROPERTY_TYPE_EXTRUSION_AXIS);
    slic3r_api::EPropertyExtrusionAxis &axis =
        extrusion.get_or_add(slic3r_api::EPropertyExtrusionAxis::key);
    axis.retract_to(2.5, true);
    CHECK(axis.operation == C_EXTRUSION_AXIS_OPERATION_RETRACT_TO);
    CHECK(axis.value == Approx(2.5));
    CHECK(axis.toolchange == 1);

    CHECK(first_key.remove(properties));
    CHECK_FALSE(first_key.has(properties));
    CHECK(second_key.has(properties));

    // A stable name may never be rebound to a different binary payload.
    CHECK_THROWS_AS(
        PluginPropertyKey<IncompatibleRuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.first"),
        std::runtime_error);
}

TEST_CASE("Extrusion visitors resolve inherited dynamic properties by key",
          "[plugins][properties][property-key]")
{
    using slic3r_api::PluginPropertyKey;
    using slic3r_api::StoredExtrusionEntity;

    const PluginPropertyKey<RuntimePropertyPayload> property_key =
        PluginPropertyKey<RuntimePropertyPayload>::register_dynamic(
            test_orchestrator_handle(), "tests.plugin-property-key.visitor");

    PluginStorage storage;
    StoredExtrusionEntity root(reinterpret_cast<storage_handle *>(&storage));
    StoredExtrusionEntity child(reinterpret_cast<storage_handle *>(&storage));
    REQUIRE_FALSE(slic3r_api::is_invalid_index(root.append_child_move(child.mutable_view())));
    property_key.get_or_add(root).value = 73;

    RuntimePropertyVisitor visitor(property_key);
    visitor.traverse(root.mutable_view());
    CHECK(visitor.visited_value == 73);
}

TEST_CASE("PluginPropertyContainer stores small typed payloads safely",
          "[plugins][properties]")
{
    const slic3r_api::PluginPropertyKey<TestSurfacePayload> property_key =
        test_surface_payload_key();

    PluginPropertyContainer container;
    slic3r_api::PluginProperties properties = property_view(container);

    // A new payload is zero-initialized, so plugins can fill only the fields
    // they care about without reading uninitialized bytes first.
    TestSurfacePayload &payload = property_key.get_or_add(properties);
    CHECK(payload.priority == 0);
    CHECK(payload.layer_count == 0);

    payload.priority = 7;
    payload.layer_count = 3;

    const TestSurfacePayload *readback = property_key.get(properties);
    REQUIRE(readback != nullptr);
    CHECK(readback->priority == 7);
    CHECK(readback->layer_count == 3);

    // A copied container owns its own bytes. Later mutations on the copy should
    // not change the original payload carried by the source object.
    PluginPropertyContainer copy = container;
    slic3r_api::PluginProperties copy_properties = property_view(copy);
    property_key.get_or_add(copy_properties).priority = 11;
    CHECK(property_key.get(properties)->priority == 7);
    CHECK(property_key.get(copy_properties)->priority == 11);

    // Reusing a type with a different layout is rejected. This catches the most
    // dangerous plugin mistake: two modules agreeing on a name but disagreeing
    // on the binary struct stored under that name.
    void *wrong_layout = container.get_or_add_property_data_mutable(
        property_key.type(), sizeof(uint32_t), alignof(uint32_t));
    CHECK(wrong_layout == nullptr);
}

TEST_CASE("Surface plugin properties survive splits and protect merge checks",
          "[plugins][properties]")
{
    const slic3r_api::PluginPropertyKey<TestSurfacePayload> property_key =
        test_surface_payload_key();

    Surface source(stPosInternal | stDensSparse, rectangle_expolygon(0., 0., 10., 10.));
    TestSurfacePayload &source_payload = property_key.get_or_add(property_view(source));
    source_payload.priority = 4;
    source_payload.layer_count = 2;

    // Splitting a Surface creates new geometry but should keep the logical
    // metadata. A dense-infill plugin, for example, can attach a priority before
    // another plugin clips the area into several pieces.
    Surface split_piece(source, rectangle_expolygon(0., 0., 5., 10.));
    const TestSurfacePayload *split_payload = property_key.get(property_view(split_piece));
    REQUIRE(split_payload != nullptr);
    CHECK(split_payload->priority == 4);
    CHECK(split_payload->layer_count == 2);

    // Surfaces may be merged only when plugin metadata matches too. Otherwise a
    // cleanup pass could accidentally fuse two areas that a later plugin expects
    // to process with different parameters.
    Surface same_metadata(source, rectangle_expolygon(5., 0., 10., 10.));
    CHECK(surfaces_could_merge(split_piece, same_metadata));

    property_key.get_or_add(property_view(same_metadata)).priority = 9;
    CHECK_FALSE(surfaces_could_merge(split_piece, same_metadata));
}

TEST_CASE("Surface runtime ids track logical ownership across copy and move",
          "[plugins][properties][surface-id]")
{
    Surface source(stPosInternal | stDensSparse, rectangle_expolygon(0., 0., 10., 10.));
    REQUIRE(source.id() != 0);

    // A copied Surface is a new infill job with copied geometry and metadata.
    // It must therefore receive a fresh runtime id instead of aliasing the
    // source surface that may generate a different extrusion subtree.
    Surface copied(source);
    CHECK(copied.id() != 0);
    CHECK(copied.id() != source.id());

    // Copy assignment replaces the destination content but keeps destination
    // identity. This protects code that already holds a reference to that
    // Surface object while a collection rewrites its geometry in-place.
    Surface assigned(stPosTop | stDensSolid, rectangle_expolygon(20., 0., 30., 10.));
    const uint64_t assigned_id = assigned.id();
    assigned = source;
    CHECK(assigned.id() == assigned_id);
    CHECK(assigned.id() != source.id());

    // Move construction and move assignment transfer the logical surface job:
    // the same geometry/properties continue under a new C++ object, so the id
    // follows the moved content.
    Surface move_construct_source(stPosBottom | stDensSolid, rectangle_expolygon(40., 0., 50., 10.));
    const uint64_t move_construct_id = move_construct_source.id();
    Surface move_constructed(std::move(move_construct_source));
    CHECK(move_constructed.id() == move_construct_id);

    Surface move_assign_source(stPosInternal | stDensSparse, rectangle_expolygon(60., 0., 70., 10.));
    const uint64_t move_assign_source_id = move_assign_source.id();
    Surface move_assigned(stPosTop | stDensSolid, rectangle_expolygon(80., 0., 90., 10.));
    move_assigned = std::move(move_assign_source);
    CHECK(move_assigned.id() == move_assign_source_id);

    // Surface vectors reallocate frequently while clipping and rebuilding
    // collections. Reallocation must move surfaces, not copy them, otherwise a
    // completely ordinary push_back could silently change ids already used by
    // generated infill subtrees.
    std::vector<Surface> collection;
    collection.reserve(1);
    collection.emplace_back(stPosInternal | stDensSparse, rectangle_expolygon(100., 0., 110., 10.));
    const uint64_t first_id_before_reallocation = collection.front().id();
    collection.emplace_back(stPosInternal | stDensSparse, rectangle_expolygon(120., 0., 130., 10.));
    CHECK(collection.front().id() == first_id_before_reallocation);
}

TEST_CASE("Support auxiliary layers are recognized through the plugin data tree API",
          "[plugins][properties][auxiliary-layers]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Model model;
    Print print;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", "0.2" },
        { "first_layer_height", "0.2" },
        { "support_material", "0" }
    });
    Slic3r::Test::init_print({Slic3r::Test::TestMesh::cube_20x20x20}, print, model, config);

    REQUIRE(print.objects().size() == 1);
    PrintObject &object = print.object(0);
    object.clear_auxiliary_layers();

    /*
    Auxiliary layers are generic storage. A newly-created auxiliary layer is
    just a normal Layer until a support generator marks it with the built-in
    support property. This distinction matters because skirt, brim or wipe
    tower layers will use the same container later without becoming support.
    */
    Layer &plain_layer = object.add_auxiliary_layer(0, scale_i(0.2), scale_i(0.2));
    CHECK(plain_layer.get_property<LayerSupportProperty>() == nullptr);

    const object_handle *object_api_handle = reinterpret_cast<const object_handle *>(&object);
    REQUIRE(object_count_auxiliary_layer(object_api_handle) == 1);
    const layer_handle *plain_handle = object_get_auxiliary_layer(object_api_handle, 0);
    REQUIRE(plain_handle != nullptr);
    const plugin_property_container_handle *plain_properties = layer_get_properties(plain_handle);
    REQUIRE(plain_properties != nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == 0);
    CHECK(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == 0);
    CHECK(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == 0);
    CHECK(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == 0);
    CHECK(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == 0);
    CHECK(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == 0);
    CHECK(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == nullptr);

    const slic3r_api::Object object_view(object_api_handle);
    REQUIRE(object_view.auxiliary_layer_count() == 1);
    CHECK(object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerSupportProperty::key) == nullptr);
    CHECK(object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerBrimProperty::key) == nullptr);
    CHECK(object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerAdhesionProperty::key) == nullptr);

    LayerSupportProperty &support_property = plain_layer.get_or_add_property<LayerSupportProperty>();
    support_property.interface_id = 17;
    support_property.reserved = 0;
    REQUIRE(plain_layer.get_property<LayerSupportProperty>() != nullptr);
    CHECK(plain_layer.get_property<LayerSupportProperty>()->interface_id == 17);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) != 0);
    REQUIRE(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT) == sizeof(c_layer_support_property));
    const c_layer_support_property *raw_support_property =
        static_cast<const c_layer_support_property *>(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_SUPPORT));
    REQUIRE(raw_support_property != nullptr);
    CHECK(raw_support_property->interface_id == 17);

    const slic3r_api::LayerSupportProperty *view_support_property =
        object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerSupportProperty::key);
    REQUIRE(view_support_property != nullptr);
    CHECK(view_support_property->interface_id == 17);

    LayerBrimProperty &brim_property = plain_layer.get_or_add_property<LayerBrimProperty>();
    brim_property.reserved = 0;
    REQUIRE(plain_layer.get_property<LayerBrimProperty>() != nullptr);
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) != 0);
    REQUIRE(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM) == sizeof(c_layer_brim_property));
    const c_layer_brim_property *raw_brim_property =
        static_cast<const c_layer_brim_property *>(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_BRIM));
    REQUIRE(raw_brim_property != nullptr);

    const slic3r_api::LayerBrimProperty *view_brim_property =
        object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerBrimProperty::key);
    REQUIRE(view_brim_property != nullptr);
    CHECK(slic3r_api::layer_is_brim(object_view.auxiliary_layer(0)));

    /*
    Brim and skirt now share one adhesion marker instead of creating one
    built-in property per adhesion feature. The payload is small but explicit:
    kind selects brim/skirt, and flags mark special storage such as
    first-layer-only skirt-brim output.
    */
    LayerAdhesionProperty &adhesion_property = plain_layer.get_or_add_property<LayerAdhesionProperty>();
    adhesion_property.kind = RAW_LAYER_ADHESION_KIND_SKIRT;
    adhesion_property.flags = RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY;
    REQUIRE(plain_layer.get_property<LayerAdhesionProperty>() != nullptr);
    CHECK(plain_layer.get_property<LayerAdhesionProperty>()->is_skirt());
    CHECK(plain_layer.get_property<LayerAdhesionProperty>()->is_first_layer_only());
    CHECK(plugin_property_has(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) != 0);
    REQUIRE(plugin_property_data_size(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION) == sizeof(c_layer_adhesion_property));
    const c_layer_adhesion_property *raw_adhesion_property =
        static_cast<const c_layer_adhesion_property *>(plugin_property_data(plain_properties, PLUGIN_PROPERTY_TYPE_LAYER_ADHESION));
    REQUIRE(raw_adhesion_property != nullptr);
    CHECK(raw_adhesion_property->kind == RAW_LAYER_ADHESION_KIND_SKIRT);
    CHECK(raw_adhesion_property->flags == RAW_LAYER_ADHESION_FLAG_FIRST_LAYER_ONLY);

    const slic3r_api::LayerAdhesionProperty *view_adhesion_property =
        object_view.auxiliary_layer(0).properties().get(slic3r_api::LayerAdhesionProperty::key);
    REQUIRE(view_adhesion_property != nullptr);
    CHECK(view_adhesion_property->is_skirt());
    CHECK(view_adhesion_property->is_first_layer_only());
    CHECK(slic3r_api::layer_adhesion_property(object_view.auxiliary_layer(0)) == view_adhesion_property);
    CHECK(slic3r_api::layer_has_adhesion_kind(
        object_view.auxiliary_layer(0), RAW_LAYER_ADHESION_KIND_SKIRT));
    CHECK_FALSE(slic3r_api::layer_is_brim(object_view.auxiliary_layer(0)));
    CHECK_FALSE(slic3r_api::layer_is_normal_skirt(object_view.auxiliary_layer(0)));
    CHECK(slic3r_api::layer_is_skirt_first_layer_only(object_view.auxiliary_layer(0)));
}
