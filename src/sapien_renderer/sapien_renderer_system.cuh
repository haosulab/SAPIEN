
namespace sapien {

void transform_sapien_to_render(float **scene_transform_buffers, int *scene_indices,
                                int *transform_indices, float *local_transforms,
                                float *local_scales, int *parent_indices, float *parent_transforms,
                                int parent_transform_stride, int count, void *stream);

void transform_sapien_to_camera_view_model(float **camera_buffers, float *local_transforms,
                                           int *parent_indices, float *parent_transforms,
                                           int parent_transform_stride, int count, void *stream);

} // namespace sapien
