///|/ Copyright (c) Prusa Research 2021 - 2023 Oleksandra Iushchenko @YuSanka, Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Lukáš Hejl @hejllukas
///|/
///|/ Copyright (c) SuperSlicer 2020 - 2024 Durand Remi @supermerill
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <oneapi/tbb/parallel_for.h>

#include "ClipperUtils.hpp"
#include "Layer.hpp"
#include "Print.hpp"
#include "PrintObject.hpp"
#include "Thread.hpp"
#include "TriangleMeshSlicer.hpp"

namespace Slic3r {

LayerUPtrs new_layers(
    PrintObject                 *print_object,
    // Object layers (pairs of bottom/top Z coordinate), without the raft.
    const std::vector<double> &object_layers)
{
    LayerUPtrs out;
    out.reserve(object_layers.size());
    auto     id   = int(0);
    Layer   *prev = nullptr;
    for (size_t i_layer = 0; i_layer < object_layers.size(); i_layer += 2) {
        double lo = object_layers[i_layer];
        double hi = object_layers[i_layer + 1];
        double slice_z = 0.5 * (lo + hi);
        // Layers are stored in object-local coordinates. Raft/support code can
        // use object_print_z_min when it needs the physical print offset, but
        // raft settings must not change object layer ids or Z coordinates.
        Layer *layer = new Layer(id++, print_object, scale_to_layer_coord(hi - lo),
                                 scale_to_layer_coord(hi), slice_z, true);
        out.emplace_back(layer);
        if (prev != nullptr) {
            prev->upper_layer = layer;
            layer->lower_layer = prev;
        }
        prev = layer;
    }
    return out;
}

static std::vector<ExPolygons> slice_volume(
    const ModelVolume             &volume,
    const std::vector<float>      &zs,
    const MeshSlicingParamsEx     &params,
    const std::function<void()>   &throw_on_cancel_callback)
{
    std::vector<ExPolygons> layers;
    if (! zs.empty()) {
        indexed_triangle_set its = volume.mesh().its;
        if (its.indices.size() > 0) {
            MeshSlicingParamsEx params2 { params };
            params2.trafo = params2.trafo * volume.get_matrix();
            if (params2.trafo.rotation().determinant() < 0.)
                its_flip_triangles(its);
            layers = slice_mesh_ex(its, zs, params2, throw_on_cancel_callback);
            throw_on_cancel_callback();
        }
    }

    return layers;
}

std::vector<ExPolygons> PrintObject::slice_support_volumes(const ModelVolumeType model_volume_type) const
{
    auto it_volume     = this->model_object()->volumes.begin();
    auto it_volume_end = this->model_object()->volumes.end();
    for (; it_volume != it_volume_end && (*it_volume)->type() != model_volume_type; ++ it_volume) ;
    std::vector<ExPolygons> slices;
    if (it_volume != it_volume_end) {
        // Found at least a single support volume of model_volume_type.
        std::vector<float> zs = slice_z_from_layers(this->layers());
        std::vector<char>  merge_layers;
        bool               merge = false;
        const Print       *print = this->print();
        auto               throw_on_cancel_callback = std::function<void()>([print](){ print->throw_if_canceled(); });
        MeshSlicingParamsEx params;
        params.trafo = this->trafo_centered();
        for (; it_volume != it_volume_end; ++ it_volume)
            if ((*it_volume)->type() == model_volume_type) {
                std::vector<ExPolygons> slices2 = slice_volume(*(*it_volume), zs, params, throw_on_cancel_callback);
                if (slices.empty()) {
                    slices = std::move(slices2);
                } else if (!slices2.empty()) {
                    if (merge_layers.empty())
                        merge_layers.assign(zs.size(), false);
                    for (size_t i = 0; i < zs.size(); ++ i) {
                        if (slices[i].empty())
                            slices[i] = std::move(slices2[i]);
                        else if (! slices2[i].empty()) {
                            append(slices[i], std::move(slices2[i]));
                            merge_layers[i] = true;
                            merge = true;
                        }
                    }
                }
            }
        if (merge) {
            std::vector<ExPolygons*> to_merge;
            to_merge.reserve(zs.size());
            for (size_t i = 0; i < zs.size(); ++ i) {
                if (merge_layers[i]) {
                    to_merge.emplace_back(&slices[i]);
                }
            }
            Slic3r::parallel_for(size_t(0), to_merge.size(),
                [&to_merge](const size_t i) {
                        *to_merge[i] = union_ex(*to_merge[i]);
            });
        }
    }
    return slices;
}

} // namespace Slic3r
