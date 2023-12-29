

struct CUstream_st;

namespace sapien {

// // for body
// // data is 64 bit
// // index is 128 bit
// // offset is 16 bit
// void body_data_physx_to_sapien_subtract_offset(void* output, void *input, void *index, void *offset, int count, void *stream);
// void body_data_sapien_to_physx_add_offset(void* output, void *input, void *index, void *offset, int count, void *stream);

// // for articulation
// // data is 32 bit
// // index is 32 bit
// // offset is 16 bit
// void transform_sapien_to_physx_add_offset(void* output, void *input, int link_count, void *index, void *offset, int count, void *stream);
// void transform_physx_to_sapien_subtract_offset(void* output, void *input, int link_count, void *index, void *offset, int count, void *stream);


void body_data_physx_to_sapien(void *sapien_data, void *physx_data, void *offset, int count, CUstream_st *);
void link_pose_physx_to_sapien(void *sapien_data, void *physx_pose, void *offset, int link_count, int count, CUstream_st *);
void link_vel_physx_to_sapien(void *sapien_data, void *physx_vel, int count, CUstream_st *);

void body_data_sapien_to_physx(void *physx_data, void *sapien_data, void *offset, int count, CUstream_st *);
void body_data_sapien_to_physx(void *physx_data, void *physx_index, void *sapien_data, void *sapien_index, void *apply_index, void *offset, int count, CUstream_st *);

void root_pose_sapien_to_physx(void *physx_pose, void *sapien_data, void *index, void *offset, int link_count, int count, CUstream_st *);
void root_vel_sapien_to_physx(void *physx_vel, void *sapien_data, void *index, int link_count, int count, CUstream_st *);


}
