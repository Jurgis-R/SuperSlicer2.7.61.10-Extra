///|/ Copyright (c) SuperSlicer 2026 Durand Remi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
/* Compile the complete public contract as C and exercise native linkage. */
#include "libslic3r/Api/plugin/c/slic3r_def.h"
#include "libslic3r/Api/plugin/c/slic3r_mesh_queries.h"
#ifdef __cplusplus
#error This fixture must be compiled as C.
#endif

typedef char extruder_index_size_must_be_16_bits[sizeof(extruder_index_t) == 2 ? 1 : -1];
typedef char extruder_index_must_be_signed[((extruder_index_t)-1 < 0) ? 1 : -1];
typedef char extruder_unknown_must_be_minus_one[
    EXTRUDER_ID_UNKNOWN == (extruder_index_t)-1 ? 1 : -1];

int test_mesh_queries_c(void) {
    mesh_query_diagnostic diagnostic;
    mesh_query_mesh *mesh = 0;
    mesh_query_status status = mesh_query_mesh_create(&mesh, &diagnostic);
    if (status != MESH_QUERY_OK || !mesh)
        return 0;
    mesh_query_mesh_destroy(mesh);
    return 1;
}
