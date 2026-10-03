
#include "StepPrepareForPeriemters.hpp"

#include "libslic3r/Api/plugin/c/steps/slic3r_step_pre_perimeter.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintObject.hpp"

#include "StepRunner.hpp"

namespace Slic3r::Steps::StepPrepareForPeriemters {

void clear(Print &print)
{
    for (PrintObject &object : print.objects())
        object.clear_auxiliary_layers();
    // Print-level auxiliaries have a hidden owner and are not included in print.objects().
    print.reset_auxiliary_object();
}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string &)
{
    return true;
}

bool validate_post(const Print &, std::string &)
{
    return true;
}

void run_step(Orchestrator &orchestrator, Print &print)
{
    Detail::run_object_step_plugins(orchestrator,
                                    print,
                                    STEP_PRE_PERIMETER,
                                    &print.full_print_config(),
                                    print.objects().size(),
                                    [&print](const size_t object_idx) {
        run_ctx_prepare_for_perimeters context = {};
        context.print = reinterpret_cast<const print_handle *>(&print);
        context.object = reinterpret_cast<const object_handle *>(&print.object(object_idx));
        return context;
    });
}

} // namespace Slic3r::Steps::StepPrepareForPeriemters
