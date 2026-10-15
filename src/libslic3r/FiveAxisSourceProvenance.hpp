///|/ Five-axis source provenance carried through slice-region construction.
///|/ Copyright (c) SuperSlicer contributors.
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher.
#ifndef slic3r_FiveAxisSourceProvenance_hpp_
#define slic3r_FiveAxisSourceProvenance_hpp_

#include "ObjectID.hpp"
#include <algorithm>
#include <vector>

namespace Slic3r {

enum class FiveAxisProvenanceState {
    Unknown,     // No trustworthy source attribution is available.
    Unique,      // Geometry is attributable to exactly one source volume.
    Mixed,       // Geometry combines contributions from multiple source volumes.
    Ambiguous    // Geometry-changing operations prevent safe attribution.
};

// Conservative metadata for five-axis planning. This does not alter ordinary slicing.
// Callers must not treat Mixed or Ambiguous as a single-volume source.
struct FiveAxisSourceProvenance {
    FiveAxisProvenanceState state { FiveAxisProvenanceState::Unknown };
    std::vector<ObjectID> source_volume_ids;

    static FiveAxisSourceProvenance unique(ObjectID id) {
        FiveAxisSourceProvenance out;
        if (id.valid()) {
            out.source_volume_ids.push_back(id);
            out.state = FiveAxisProvenanceState::Unique;
        }
        return out;
    }

    bool is_unique() const {
        return state == FiveAxisProvenanceState::Unique && source_volume_ids.size() == 1;
    }

    void mark_ambiguous() { state = FiveAxisProvenanceState::Ambiguous; }

    void merge(const FiveAxisSourceProvenance &rhs) {
        const bool ambiguous = state == FiveAxisProvenanceState::Ambiguous ||
                               rhs.state == FiveAxisProvenanceState::Ambiguous;
        for (const ObjectID id : rhs.source_volume_ids)
            if (std::find(source_volume_ids.begin(), source_volume_ids.end(), id) == source_volume_ids.end())
                source_volume_ids.push_back(id);

        if (ambiguous) {
            state = FiveAxisProvenanceState::Ambiguous;
        } else if (source_volume_ids.empty()) {
            state = FiveAxisProvenanceState::Unknown;
        } else if (source_volume_ids.size() > 1 ||
                   state == FiveAxisProvenanceState::Mixed ||
                   rhs.state == FiveAxisProvenanceState::Mixed) {
            state = FiveAxisProvenanceState::Mixed;
        } else {
            state = FiveAxisProvenanceState::Unique;
        }
    }
};

} // namespace Slic3r
#endif
