
#include "StepGenerateSupport.hpp"

#include "StepSupportDemand.hpp"

namespace Slic3r::Steps::StepGenerateSupport {

void clear(Print &) {}
void prepare(Print &) {}

bool validate_pre(const Print &, std::string *)
{
    return true;
}

bool validate_post(const Print &, std::string *)
{
    return true;
}

void run_step(Orchestrator &, Print &) {}

void run_step(Orchestrator &orchestrator, Print &print, const StepSupportDemand::State &)
{
    run_step(orchestrator, print);
}

} // namespace Slic3r::Steps::StepGenerateSupport
