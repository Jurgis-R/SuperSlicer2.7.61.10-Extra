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
///|/ Copyright (c) 2012 Henrik Brix Andersen
///|/ Copyright (c) 2012 Michael Moon
///|/ Copyright (c) 2011 Richard Goodwin
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_PrintRegion_hpp_
#define slic3r_PrintRegion_hpp_

#include <cstddef>
#include <cstdint>
#include <set>

#include "Flow.hpp"
#include "libslic3r.h"
#include "Config/FFFPrintConfig.hpp"

namespace Slic3r {

class Print;
class PrintObject;

// Regional configuration snapshot. Print owns registered snapshots until clear();
// object/layer associations borrow them and keep their own dense slicing indices.
class PrintRegion
{
public:
    PrintRegion() : PrintRegion(PrintRegionConfig{}) {}
    PrintRegion(const PrintRegionConfig &config);
    PrintRegion(const PrintRegionConfig &config, const size_t config_hash);
    PrintRegion(PrintRegionConfig &&config);
    PrintRegion(PrintRegionConfig &&config, const size_t config_hash);
    ~PrintRegion() = default;

// Methods NOT modifying the PrintRegion's state:
public:
    const PrintRegionConfig&    config() const throw() { return m_config; }
    size_t                      config_hash() const throw() { return m_config_hash; }
	// 1-based extruder identifier for this region and role.
    uint16_t 				    extruder(FlowRole role, const PrintObject& object) const;
    Flow                        flow(const PrintObject &object, FlowRole role, double layer_height, size_t layer_id) const;
    float                       width(FlowRole role, bool first_layer, const PrintObject& object) const;
    // Average diameter of nozzles participating on extruding this region.
    coordf_t                    nozzle_dmr_avg(const PrintConfig &print_config) const;

    // Collect 0-based extruder indices used to print this region's object.
	void                        collect_object_printing_extruders(const Print& print, std::set<uint16_t> &object_extruders) const;
	static void                 collect_object_printing_extruders(const PrintConfig &print_config, const PrintObjectConfig &object_config, const PrintRegionConfig &region_config, std::set<uint16_t> &object_extruders);

private:
    // Immutable snapshot: copies belong to this region, never to its caller.
    const PrintRegionConfig m_config;
    const size_t m_config_hash;

};

inline bool operator==(const PrintRegion &lhs, const PrintRegion &rhs) { return lhs.config_hash() == rhs.config_hash() && lhs.config() == rhs.config(); }
inline bool operator!=(const PrintRegion &lhs, const PrintRegion &rhs) { return ! (lhs == rhs); }

} // namespace Slic3r

#endif // slic3r_PrintRegion_hpp_
