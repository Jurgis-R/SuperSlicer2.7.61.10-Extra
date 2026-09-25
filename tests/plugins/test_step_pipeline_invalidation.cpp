#include <catch2/catch.hpp>

#include <algorithm>

#include "plugin_test_helpers.hpp"

#include "libslic3r/Print.hpp"
#include "libslic3r/Printing/PrintingPlan.hpp"
#include "libslic3r/Steps/StepPipeline.hpp"

namespace {

bool contains_step(const std::vector<slicing_step_t> &steps, slicing_step_t step)
{
    return std::find(steps.begin(), steps.end(), step) != steps.end();
}

} // namespace

TEST_CASE("Step pipeline DAG invalidates only skirt/brim dependents", "[plugins][step-pipeline][invalidation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The new pipeline is not topologically sorted at runtime. This test
    // protects the central dependency table used by Print invalidation: changing
    // skirt/brim must re-run its finalization consumers, but it must not force
    // upstream support producers or unrelated perimeter, surface and infill steps.
    const std::vector<slicing_step_t> dependents = Slic3r::Steps::dependent_steps_closure(STEP_SKIRT_BRIM);

    CHECK_FALSE(contains_step(dependents, STEP_SUPPORT_DEMAND));
    CHECK_FALSE(contains_step(dependents, STEP_SUPPORT));
    CHECK(contains_step(dependents, STEP_FINALIZE_SLICING));
    CHECK(contains_step(dependents, STEP_ORDERING));
    CHECK(contains_step(dependents, STEP_WIPETOWER));
    CHECK(contains_step(dependents, STEP_GCODE));

    CHECK_FALSE(contains_step(dependents, STEP_PERIMETER));
    CHECK_FALSE(contains_step(dependents, STEP_SURFACE_GENERATION));
    CHECK_FALSE(contains_step(dependents, STEP_INFILL));
    CHECK_FALSE(contains_step(dependents, STEP_POST_INFILL));

    // A fresh Print starts with a full run requested. Marking every known step
    // as executed gives the same empty plan that a successful process() leaves
    // behind, then the single-step invalidation should rebuild only the
    // skirt/brim branch and its declared downstream consumers.
    Slic3r::Print print;
    for (slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);

    print.mark_step_and_dependents_for_execution(STEP_SKIRT_BRIM);

    CHECK(print.should_execute_step(STEP_SKIRT_BRIM));
    CHECK(print.should_execute_step(STEP_FINALIZE_SLICING));
    CHECK(print.should_execute_step(STEP_ORDERING));
    CHECK(print.should_execute_step(STEP_GCODE));

    CHECK_FALSE(print.should_execute_step(STEP_SUPPORT_DEMAND));
    CHECK_FALSE(print.should_execute_step(STEP_SUPPORT));
    CHECK_FALSE(print.should_execute_step(STEP_PERIMETER));
    CHECK_FALSE(print.should_execute_step(STEP_SURFACE_GENERATION));
    CHECK_FALSE(print.should_execute_step(STEP_INFILL));
    CHECK_FALSE(print.should_execute_step(STEP_POST_INFILL));
}

TEST_CASE("Step pipeline dependency order matches written execution order", "[plugins][step-pipeline][invalidation]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    // The dependency graph is only an invalidation graph. Runtime still follows
    // StepPipeline::execution_order(), so every edge must point forward in that
    // written order or a dependent step could be requested before its producer.
#ifdef _DEBUG
    CHECK(Slic3r::Steps::validate_execution_order_against_dependencies());
#else
    SUCCEED("Dependency order validation is a debug-only guard.");
#endif
}

TEST_CASE("Every executable step resumes from the last checkpoint before it",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;

    const std::vector<std::pair<slicing_step_t, Checkpoint>> expected {
        { STEP_LAYER_HEIGHT, Checkpoint::None },
        { STEP_SLICING, Checkpoint::None },
        { STEP_POST_SLICING, Checkpoint::None },
        { STEP_PRE_PERIMETER, Checkpoint::Slicing },
        { STEP_PERIMETER, Checkpoint::Slicing },
        { STEP_POST_PERIMETER, Checkpoint::Slicing },
        { STEP_SURFACE_GENERATION, Checkpoint::Slicing },
        { STEP_PRE_INFILL, Checkpoint::Slicing },
        { STEP_INFILL_GROUP, Checkpoint::Slicing },
        { STEP_INFILL, Checkpoint::Slicing },
        { STEP_POST_INFILL, Checkpoint::Slicing },
        { STEP_SUPPORT_DEMAND, Checkpoint::Slicing },
        { STEP_SUPPORT, Checkpoint::Slicing },
        { STEP_SKIRT_BRIM, Checkpoint::Slicing },
        { STEP_FINALIZE_SLICING, Checkpoint::Slicing },
        { STEP_ORDERING, Checkpoint::Print },
        { STEP_WIPETOWER, Checkpoint::Ordering },
        { STEP_SUPPORT_SPOT, Checkpoint::Ordering },
        { STEP_LAYER_EXTRUSION_EDIT, Checkpoint::Ordering },
        { STEP_LAYER_STICHING, Checkpoint::Ordering },
        { STEP_EXTRUSION_EDIT, Checkpoint::Ordering },
        { STEP_EXTRUSION_SIMPLIFICATION, Checkpoint::Ordering },
        { STEP_GCODE, Checkpoint::Plan }
    };

    REQUIRE(expected.size() == Slic3r::Steps::execution_order().size());
    for (size_t index = 0; index < expected.size(); ++index) {
        CHECK(expected[index].first == Slic3r::Steps::execution_order()[index]);
        CHECK(Slic3r::Steps::checkpoint_before_step(expected[index].first) ==
              expected[index].second);
    }
    CHECK(Slic3r::Steps::checkpoint_before_step(STEP_ANY) == Checkpoint::None);

    // Each checkpoint resumes at the first step it does not contain.
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::None) == STEP_LAYER_HEIGHT);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Slicing) == STEP_PRE_PERIMETER);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Print) == STEP_ORDERING);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Ordering) == STEP_WIPETOWER);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::Plan) == STEP_GCODE);
    CHECK(Slic3r::Steps::checkpoint_resume_step(Checkpoint::GCode) == STEP_NONE);

    // Numeric ABI values deliberately disagree with the execution order here.
    CHECK(STEP_SKIRT_BRIM < STEP_SUPPORT);
    CHECK(Slic3r::Steps::executes_before(STEP_SUPPORT, STEP_SKIRT_BRIM));
}

TEST_CASE("A setting read by a plugin invalidates no later than that plugin",
          "[plugins][step-pipeline][invalidation][config]")
{
    using Slic3r::Steps::tightened_invalidation_step;

    // A full replay and an explicit no-replay declaration are kept as is.
    CHECK(tightened_invalidation_step(STEP_ANY, STEP_INFILL) == STEP_ANY);
    CHECK(tightened_invalidation_step(STEP_NONE, STEP_INFILL) == STEP_NONE);

    // An earlier or equal step already replays the reader.
    CHECK(tightened_invalidation_step(STEP_PERIMETER, STEP_INFILL) == STEP_PERIMETER);
    CHECK(tightened_invalidation_step(STEP_INFILL, STEP_INFILL) == STEP_INFILL);
    // A later legacy step moves back to the reader.
    CHECK(tightened_invalidation_step(STEP_GCODE, STEP_INFILL) == STEP_INFILL);

    // Support runs before skirt/brim although its ABI value is larger.
    CHECK(tightened_invalidation_step(STEP_SKIRT_BRIM, STEP_SUPPORT) == STEP_SUPPORT);
    CHECK(tightened_invalidation_step(STEP_SUPPORT, STEP_SKIRT_BRIM) == STEP_SUPPORT);
}

TEST_CASE("Replay requests resume from the earliest invalid checkpoint",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;

    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    CHECK(print.current_checkpoint() == Checkpoint::GCode);

    // Resuming from the final checkpoint forces nothing.
    print.request_replay_from(Checkpoint::GCode);
    CHECK(print.current_checkpoint() == Checkpoint::GCode);

    // The plan checkpoint forces at least the G-code writer.
    print.request_replay_from(Checkpoint::Plan);
    CHECK(print.current_checkpoint() == Checkpoint::Plan);
    CHECK(print.should_execute_step(STEP_GCODE));
    CHECK_FALSE(print.should_execute_step(STEP_EXTRUSION_SIMPLIFICATION));
    CHECK(print.printing_plan() != nullptr);

    // An earlier invalid checkpoint wins over a later request.
    print.request_replay_from(Checkpoint::Slicing);
    CHECK(print.current_checkpoint() == Checkpoint::Slicing);
    CHECK(print.should_execute_step(STEP_PRE_PERIMETER));
    CHECK_FALSE(print.should_execute_step(STEP_POST_SLICING));
    print.request_replay_from(Checkpoint::Plan);
    CHECK(print.current_checkpoint() == Checkpoint::Slicing);

    // Resuming from the Print data tree or earlier releases the plan, which
    // borrows data that is about to be rebuilt.
    CHECK(print.printing_plan() == nullptr);
}

TEST_CASE("Plan checkpoints fall back to the Print checkpoint without their plan",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);

    // Without a live plan, STEP_GCODE has nothing to write.
    print.request_replay_from(Checkpoint::Plan);
    CHECK(print.current_checkpoint() == Checkpoint::Print);

    // With a plan but without its ordered copy, plan editors cannot restart.
    print.mutable_printing_plan().groups.emplace_back();
    print.request_replay_from(Checkpoint::Ordering);
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK(print.current_checkpoint() == Checkpoint::Print);

    // Resuming then requests STEP_ORDERING again and releases the stale plan.
    print.prepare_checkpoint_resume();
    CHECK(print.should_execute_step(STEP_ORDERING));
    CHECK(print.printing_plan() == nullptr);
}

TEST_CASE("Ordering checkpoint survives downstream invalidation only",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    using Slic3r::Steps::Checkpoint;
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Print print;
    for (const slicing_step_t step : Slic3r::Steps::execution_order())
        print.mark_step_executed(step);
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());

    // Plan editors restart from the ordered copy. Ordering itself stays done.
    print.request_replay_from(Slic3r::Steps::checkpoint_before_step(STEP_EXTRUSION_EDIT));
    CHECK(print.current_checkpoint() == Checkpoint::Ordering);
    CHECK(print.has_ordering_checkpoint());
    CHECK_FALSE(print.should_execute_step(STEP_ORDERING));
    CHECK(print.should_execute_step(STEP_WIPETOWER));
    CHECK(print.should_execute_step(STEP_GCODE));

    // A resume restores a fresh clone of the ordered copy.
    const Slic3r::Printing::PrintingPlan *edited_plan = print.printing_plan();
    print.prepare_checkpoint_resume();
    CHECK(print.printing_plan() != edited_plan);
    CHECK(print.has_ordering_checkpoint());

    // Invalidating ordering's own inputs makes the copy stale.
    print.request_replay_from(Slic3r::Steps::checkpoint_before_step(STEP_SLICING));
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK(print.should_execute_step(STEP_ORDERING));
}

TEST_CASE("Disabling the ordering checkpoint releases its copy",
          "[plugins][step-pipeline][invalidation][checkpoint]")
{
    Slic3r::Test::Plugins::ensure_plugin_test_runtime_initialized();

    Slic3r::Print print;
    print.mutable_printing_plan().groups.emplace_back();
    REQUIRE(print.save_ordering_checkpoint());
    REQUIRE(print.has_ordering_checkpoint());

    print.set_keep_ordering_checkpoint(false);
    CHECK_FALSE(print.has_ordering_checkpoint());
    CHECK_FALSE(print.save_ordering_checkpoint());
}
