///|/ Copyright (c) Prusa Research 2016 - 2023 Lukáš Matěna @lukasmatena, Enrico Turri @enricoturri1966, Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas, Filip Sykala @Jony01, Oleksandra Iushchenko @YuSanka, Vojtěch Král @vojtechkral
///|/ Copyright (c) BambuStudio 2023 manch1n @manch1n
///|/ Copyright (c) SuperSlicer 2022 Remi Durand @supermerill
///|/ Copyright (c) 2019 Bryan Smith
///|/ Copyright (c) 2017 Eyal Soha @eyal0
///|/ Copyright (c) Slic3r 2013 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2017 Joseph Lenox @lordofhyphens
///|/
///|/ ported from lib/Slic3r/Print.pm:
///|/ Copyright (c) Prusa Research 2016 - 2018 Vojtěch Bubník @bubnikv, Tomáš Mészáros @tamasmeszaros
///|/ Copyright (c) Slic3r 2011 - 2016 Alessandro Ranellucci @alranel
///|/ Copyright (c) 2012 - 2013 Mark Hindess
///|/ Copyright (c) 2013 Devin Grady
///|/ Copyright (c) 2012 - 2013 Mike Sheldrake @mesheldrake
///|/ Copyright (c) 2012 Henrik Brix Andersen @henrikbrixandersen
///|/ Copyright (c) 2012 Michael Moon
///|/ Copyright (c) 2011 Richard Goodwin
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_PrintObjectRegion_hpp_
#define slic3r_PrintObjectRegion_hpp_

#include <algorithm>

#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "DataTreeFwd.hpp"
#include "libslic3r.h"
#include "ObjectID.hpp"
#include "Point.hpp"
#include "PrintRegion.hpp"
#include "SupportSpotsGenerator.hpp"

namespace Slic3r {

class PrintObject;

class PrintObjectRegions
{
public:
    // Bounding box of a ModelVolume transformed into the working space of a PrintObject, possibly
    // clipped by a layer range modifier.
    // Only Eigen types of Nx16 size are vectorized. This bounding box will not be vectorized.
    static_assert(sizeof(Eigen::AlignedBox<float, 3>) == 24, "Eigen::AlignedBox<float, 3> is not being vectorized, thus it does not need to be aligned");
    using BoundingAlignedBox3f = Eigen::AlignedBox<float, 3>;
    struct VolumeExtents {
        ObjectID             volume_id;
        BoundingAlignedBox3f          bbox;
    };

    struct VolumeRegion
    {
        // ID of the associated ModelVolume.
        const ModelVolume   *model_volume { nullptr };
        // Index of a parent VolumeRegion.
        int                  parent { -1 };
        // Borrowed Print-owned snapshot; null for a negative volume.
        const PrintRegion   *region { nullptr };
        // Pointer to VolumeExtents::bbox.
        const BoundingAlignedBox3f   *bbox { nullptr };
        // To speed up merging of same regions.
        const VolumeRegion  *prev_same_region { nullptr };
        int region_idx { -1 }; // Local all_regions index; -1 for subtractors.
    };

    struct PaintedRegion
    {
        // 1-based extruder identifier.
        unsigned int     extruder_id;
        // Index of a parent VolumeRegion.
        int              parent { -1 };
        // Borrowed Print-owned snapshot.
        const PrintRegion *region { nullptr };
        int region_idx { -1 }; // Local all_regions index.
    };

    // One slice over the PrintObject (possibly the whole PrintObject) and a list of ModelVolumes and their bounding boxes
    // possibly clipped by the layer_height_range.
    struct LayerRangeRegions
    {
        std::pair<coord_t, coord_t> layer_height_range_;
        // Config of the layer range, null if there is just a single range with no config override.
        // Config is owned by the associated ModelObject.
        const DynamicPrintConfig*   config { nullptr };
        // Volumes sorted by ModelVolume::id().
        std::vector<VolumeExtents>  volumes;

        // Sorted in the order of their source ModelVolumes, thus reflecting the order of region clipping, modifier overrides etc.
        std::vector<VolumeRegion>   volume_regions;
        std::vector<PaintedRegion>  painted_regions;

        bool has_volume(const ObjectID id) const;
    };

    struct GeneratedSupportPoints{
        Transform3d object_transform; // for frontend object mapping
        SupportSpotsGenerator::SupportPoints support_points;
        SupportSpotsGenerator::PartialObjects partial_objects;
    };

    // Borrowed canonical regions, ordered locally (fallback first).
    std::vector<const PrintRegion*> all_regions;

    // Dense index belongs to the object association, never to the shared region.
    int region_index(const PrintRegion *region) const {
        const std::vector<const PrintRegion*>::const_iterator it =
            std::find(all_regions.begin(), all_regions.end(), region);
        return it == all_regions.end() ? -1 : int(it - all_regions.begin());
    }
    std::vector<LayerRangeRegions>              layer_ranges;
    // Transformation of this ModelObject into one of the associated PrintObjects (all PrintObjects derived from a single modelObject differ by a Z rotation only).
    // This transformation is used to calculate VolumeExtents.
    Transform3d                                 trafo_bboxes;
    std::vector<ObjectID>                       cached_volume_ids;

    std::optional<GeneratedSupportPoints> generated_support_points;

    void clear();

private:
    friend class PrintObject;
    // Number of PrintObjects generated from the same ModelObject and sharing the regions.
    // ref_cnt could only be modified by the main thread, thus it does not need to be atomic.
    size_t                                      m_ref_cnt{ 0 };
};

} // namespace Slic3r

#endif // slic3r_PrintObjectRegion_hpp_
