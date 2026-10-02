#include <catch2/catch.hpp>
#include <test_utils.hpp>

#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Config/FFFPrintConfig.hpp"
#include "libslic3r/Format/SLAArchiveFormatRegistry.hpp"
#include "libslic3r/Format/SLAArchiveWriter.hpp"
#include "libslic3r/Format/SLAArchiveReader.hpp"

#include <boost/filesystem.hpp>

using namespace Slic3r;

TEST_CASE("Archive export test", "[sla_archives]") {
    // Static SLA configs require definitions and their cache before construction.
    PrintConfigDef::instance_mutable().init_common_params();
    init_fff_params(PrintConfigDef::instance_mutable());
    init_sla_params(PrintConfigDef::instance_mutable());
    initialize_fff_print_config_cache();
    initialize_sla_print_config_cache();
    auto registry = registered_sla_archives();

    for (const char * pname : {"20mm_cube", "extruder_idler"})
    for (const auto &[format, entry] : registry) {
        INFO(std::string("Testing archive type: ") + entry.id + " -- writing...");
        SLAPrint print;
        SLAFullPrintConfig fullcfg;

        auto m = Model::read_from_file(TEST_DATA_DIR PATH_SEPARATOR + std::string(pname) + ".obj", nullptr);

        fullcfg.printer_technology.set_int(ptSLA); // FIXME this should be ensured
        fullcfg.output_format.set_int(format);
        fullcfg.set("supports_enable", false);
        fullcfg.set("pad_enable", false);

        DynamicPrintConfig cfg;
        cfg.apply(fullcfg);

        print.set_status_callback([](const PrintBase::SlicingStatus&) {});
        print.apply(m, cfg);
        print.process();

        ThumbnailsList thumbnails;
        auto outputfname = std::string("output_") + pname + "." + entry.ext;

        print.export_print(outputfname, thumbnails, pname);

        // Not much can be checked about the archives...
        REQUIRE(boost::filesystem::exists(outputfname));

        double vol_written = m.mesh().volume();

        if (entry.rdfactoryfn) {
            INFO(std::string("Testing archive type: ") + entry.id + " -- reading back...");
            indexed_triangle_set its;
            DynamicPrintConfig cfg;

            try {
                // Unknown format retains extension-based reader detection.
                import_sla_archive(outputfname, ofUnknown, its, cfg);
            } catch (...) {
                REQUIRE(false);
            }

            // its_write_obj(its, (outputfname + ".obj").c_str());

            REQUIRE(!cfg.empty());
            REQUIRE(!its.empty());

            double vol_read = its_volume(its);
            double rel_err  = std::abs(vol_written - vol_read) / vol_written;
            REQUIRE(rel_err < 0.1);
        }
    }
}
