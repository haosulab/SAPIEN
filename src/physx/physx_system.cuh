
namespace sapien {

// for body
// data is 64 bit
// index is 128 bit
// offset is 16 bit
void body_data_physx_to_sapien_subtract_offset(void* output, void *input, void *index, void *offset, int count, void *stream);
void body_data_sapien_to_physx_add_offset(void* output, void *input, void *index, void *offset, int count, void *stream);

// for articulation
// data is 32 bit
// index is 32 bit
// offset is 16 bit
void transform_sapien_to_physx_add_offset(void* output, void *input, void *index, void *offset, int count, void *stream);
void transform_physx_to_sapien_subtract_offset(void* output, void *input, void *index, void *offset, int count, void *stream);

}
