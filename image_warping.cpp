#include <cmath>
#include <iostream>
#include "image_warping.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;
const float PI = 3.14159265358979f;

void cal_trans_matrix(float roll, float pitch, float yaw, uint16_t image_width, uint16_t image_height, float *trans_matrix)
{
    // rotation matrix
    Mat RX, RY, RZ, trans_matrix_mat;
    Mat A1, A2, T, R;
    float theta, phi, gamma;
    float d, focal;

    // convert degree to rad
    gamma = roll * PI / 180.0;
    theta = pitch * PI / 180.0;
    phi = yaw * PI / 180.0;

    d = sqrt(image_height * image_height + image_width * image_width);
    focal = gamma != 0 ? (d / (2 * sin(gamma))) : d;

    A1 = (Mat_<float>(4, 3) << 1.0, 0, -0.5 * image_width,
          0, 1, -0.5 * image_height,
          0, 0, 1,
          0, 0, 1);

    RX = (Mat_<float>(4, 4) << 1.0, 0, 0, 0,
          0, cos(theta), -sin(theta), 0,
          0, sin(theta), cos(theta), 0,
          0, 0, 0, 1);

    RY = (Mat_<float>(4, 4) << cos(phi), 0, -sin(phi), 0,
          0, 1, 0, 0,
          sin(phi), 0, cos(phi), 0,
          0, 0, 0, 1);

    RZ = (Mat_<float>(4, 4) << cos(gamma), -sin(gamma), 0, 0,
          sin(gamma), cos(gamma), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1);

    T = (Mat_<float>(4, 4) << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, focal,
         0, 0, 0, 1);

    R = ((RX * RY) * RZ);

    A2 = (Mat_<float>(3, 4) << focal, 0, 0.5 * image_width, 0,
          0, focal, 0.5 * image_height, 0,
          0, 0, 1, 0);

    trans_matrix_mat = A2 * (T * (R * A1));

    memcpy(trans_matrix, trans_matrix_mat.data, 3 * 3 * sizeof(float));

    // reference : https://github.com/eborboihuc/rotate_3d
}

void warp_rgb_image(float *trans_matrix, uint8_t *image, uint16_t image_width,
                    uint16_t image_height, uint8_t *warp_image)
{

    float x, y, z, idx_z;
    uint16_t x1, x2, y1, y2;
    uint8_t p11, p12, p22, p21;
    float dx1, dx2, dy1, dy2;

    float tmp1, tmp2;

    for (int i = 0; i < image_height; i++)
    {
        for (int j = 0; j < image_width; j++)
        {
            x = trans_matrix[0] * j + trans_matrix[1] * i + trans_matrix[2];
            y = trans_matrix[3] * j + trans_matrix[4] * i + trans_matrix[5];
            z = trans_matrix[6] * j + trans_matrix[7] * i + trans_matrix[8];

            x = x / z;
            y = y / z;

            x1 = floor(x);
            y1 = floor(y);

            x2 = ceil(x);
            y2 = ceil(y);

            dx1 = x - x1;
            dx2 = 1.0 - dx1;

            dy1 = y - y1;
            dy2 = 1.0 - dy1;

            // intepolate r
            if (x1 > 0 && x1 < image_width && x2 > 0 && x2 < image_width && y1 > 0 && y1 < image_height && y2 > 0 && y2 < image_height)
            {
                p11 = image[3 * (y1 * image_width + x1)];
                p21 = image[3 * (y2 * image_width + x1)];
                p12 = image[3 * (y1 * image_width + x2)];
                p22 = image[3 * (y2 * image_width + x2)];

                tmp1 = dx2 * p11 + dx1 * p12;
                tmp2 = dx2 * p21 + dx1 * p22;

                warp_image[3 * (i * image_width + j)] = (uint8_t)(dy2 * tmp1 + dy1 * tmp2);

                // intepolate g
                p11 = image[3 * (y1 * image_width + x1) + 1];
                p21 = image[3 * (y2 * image_width + x1) + 1];
                p12 = image[3 * (y1 * image_width + x2) + 1];
                p22 = image[3 * (y2 * image_width + x2) + 1];
                tmp1 = dx2 * p11 + dx1 * p12;
                tmp2 = dx2 * p21 + dx1 * p22;

                warp_image[3 * (i * image_width + j) + 1] = (uint8_t)(dy2 * tmp1 + dy1 * tmp2);

                // intepolate b
                p11 = image[3 * (y1 * image_width + x1) + 2];
                p21 = image[3 * (y2 * image_width + x1) + 2];
                p12 = image[3 * (y1 * image_width + x2) + 2];
                p22 = image[3 * (y2 * image_width + x2) + 2];
                tmp1 = dx2 * p11 + dx1 * p12;
                tmp2 = dx2 * p21 + dx1 * p22;
                warp_image[3 * (i * image_width + j) + 2] = (uint8_t)(dy2 * tmp1 + dy1 * tmp2);
            }
        }
    }
}

void convert_trans_mat_to_fixed(float *trans_matrix, int32_t trans_matrix_fixed[16])
{
    // convert trans_matrix to fixed
    for (int i = 0; i < 9; i++)
    {
        trans_matrix_fixed[i] = trans_matrix[i] * (float)(1 << TRAN_MAT_FRACT_BITS);
    }
}
void warp_rgb_image_fixed(int32_t* trans_matrix_fixed, uint8_t *image, uint16_t image_width,
                          uint16_t image_height, uint8_t *warp_image)
{

    int32_t x, y, z;
    uint32_t x1, x2, y1, y2;
    uint8_t p11, p12, p22, p21;
    uint32_t dx1, dx2, dy1, dy2;
    uint64_t tmp1, tmp2;
    double inv_z;
    uint32_t fixed_inv_z;
    

    for (int32_t i = 0; i < image_height; i++)
    {
        for (int32_t j = 0; j < image_width; j++)
        {
            x = trans_matrix_fixed[0] * j + trans_matrix_fixed[1] * i + trans_matrix_fixed[2];
            y = trans_matrix_fixed[3] * j + trans_matrix_fixed[4] * i + trans_matrix_fixed[5];
            z = trans_matrix_fixed[6] * j + trans_matrix_fixed[7] * i + trans_matrix_fixed[8];

            x1 = x / z;
            y1 = y / z;

            if (x == x1 * z)
            {
                x2 = x1;
            }
            else
            {
                x2 = x1 + 1;
            }

            if (y == y1 * z)
            {
                y2 = y1;
            }
            else
            {
                y2 = y1 + 1;
            }

            dx1 = x - x1 * z;
            dx2 = z - dx1;

            dy1 = y - y1 * z;
            dy2 = z - dy1;

            inv_z = (1.0 / z);

            fixed_inv_z = inv_z * (1 << Z_FRACT_BITS);
            // intepolate r
            if (x1 > 0 && x1 < image_width && x2 > 0 && x2 < image_width && y1 > 0 && y1 < image_height && y2 > 0 && y2 < image_height)
            {
                p11 = image[3 * (y1 * image_width + x1)];
                p21 = image[3 * (y2 * image_width + x1)];
                p12 = image[3 * (y1 * image_width + x2)];
                p22 = image[3 * (y2 * image_width + x2)];

                tmp1 = (((uint64_t)fixed_inv_z * (uint64_t)(dx2 * p11 + dx1 * p12))) >> Z_FRACT_BITS;
                tmp2 = (((uint64_t)fixed_inv_z * (uint64_t)(dx2 * p21 + dx1 * p22))) >> Z_FRACT_BITS;
                warp_image[3 * (i * image_width + j)] = (uint8_t)(((uint64_t)fixed_inv_z * (uint64_t)(dy2 * tmp1 + dy1 * tmp2)) >> Z_FRACT_BITS);

                // intepolate g
                p11 = image[3 * (y1 * image_width + x1) + 1];
                p21 = image[3 * (y2 * image_width + x1) + 1];
                p12 = image[3 * (y1 * image_width + x2) + 1];
                p22 = image[3 * (y2 * image_width + x2) + 1];
                tmp1 = (((uint64_t)fixed_inv_z * (uint64_t)(dx2 * p11 + dx1 * p12))) >> Z_FRACT_BITS;
                tmp2 = (((uint64_t)fixed_inv_z * (uint64_t)(dx2 * p21 + dx1 * p22))) >> Z_FRACT_BITS;
                warp_image[3 * (i * image_width + j) + 1] = (uint8_t)(((uint64_t)fixed_inv_z * (uint64_t)(dy2 * tmp1 + dy1 * tmp2)) >> Z_FRACT_BITS);

                // intepolate b
                p11 = image[3 * (y1 * image_width + x1) + 2];
                p21 = image[3 * (y2 * image_width + x1) + 2];
                p12 = image[3 * (y1 * image_width + x2) + 2];
                p22 = image[3 * (y2 * image_width + x2) + 2];
                tmp1 = (((uint64_t)fixed_inv_z * (uint64_t)(dx2 * p11 + dx1 * p12))) >> Z_FRACT_BITS;
                tmp2 = (((uint64_t)fixed_inv_z * (uint64_t)(dx2 * p21 + dx1 * p22))) >> Z_FRACT_BITS;
                warp_image[3 * (i * image_width + j) + 2] = (uint8_t)(((uint64_t)fixed_inv_z * (uint64_t)(dy2 * tmp1 + dy1 * tmp2)) >> Z_FRACT_BITS);
            }
        }
    }
}
