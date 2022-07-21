#pragma once
#include <stdint.h>

#if defined(_MSC_VER)
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

#define TRAN_MAT_FRACT_BITS 10
#define Z_FRACT_BITS 30

EXPORT void cal_trans_matrix(float roll, float pitch, float yaw, uint16_t image_width, uint16_t image_height, float *trans_matrix);

EXPORT void warp_rgb_image(float *trans_matrix, uint8_t *image, uint16_t image_width,
                           uint16_t image_height, uint8_t *warp_image);

EXPORT void warp_rgb_image_fixed(int32_t* trans_matrix_fixed, uint8_t *image, uint16_t image_width,
                           uint16_t image_height, uint8_t *warp_image);


EXPORT void convert_trans_mat_to_fixed(float *trans_matrix, int32_t trans_matrix_fixed[16]);