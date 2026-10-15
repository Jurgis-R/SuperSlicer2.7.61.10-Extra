#include "libslic3r/FiveAxisSourceProvenance.hpp"

#include <catch2/catch.hpp>

using namespace Slic3r;

TEST_CASE("Five-axis source provenance keeps unique attribution until distinct sources merge", "[FiveAxisSourceProvenance]")
{
    const ObjectID a(101);
    const ObjectID b(202);
    auto provenance = FiveAxisSourceProvenance::unique(a);
    REQUIRE(provenance.is_unique());
    CHECK(provenance.source_volume_ids.size() == 1);
    CHECK(provenance.source_volume_ids.front() == a);

    provenance.merge(FiveAxisSourceProvenance::unique(a));
    CHECK(provenance.is_unique());

    provenance.merge(FiveAxisSourceProvenance::unique(b));
    CHECK(provenance.state == FiveAxisProvenanceState::Mixed);
    CHECK_FALSE(provenance.is_unique());
    CHECK(provenance.source_volume_ids.size() == 2);
}

TEST_CASE("Five-axis source provenance never clears explicit ambiguity", "[FiveAxisSourceProvenance]")
{
    auto provenance = FiveAxisSourceProvenance::unique(ObjectID(101));
    provenance.mark_ambiguous();
    provenance.merge(FiveAxisSourceProvenance::unique(ObjectID(202)));
    CHECK(provenance.state == FiveAxisProvenanceState::Ambiguous);
    CHECK_FALSE(provenance.is_unique());
}

TEST_CASE("Five-axis source provenance treats invalid IDs as unknown", "[FiveAxisSourceProvenance]")
{
    const auto provenance = FiveAxisSourceProvenance::unique(ObjectID());
    CHECK(provenance.state == FiveAxisProvenanceState::Unknown);
    CHECK_FALSE(provenance.is_unique());
    CHECK(provenance.source_volume_ids.empty());
}
