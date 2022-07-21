#include <stdio.h>
#include <time.h>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "image_warping.h"
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    Mat image;
    uint8_t *warp_image;
    uint16_t image_width, image_height;
    double elapsed;
    clock_t start, end;
    uint16_t num_of_tests = 100;

    float trans_matrix[9];
    int32_t trans_matrix_fixed[16];

    if (argc != 2)
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    image = imread(argv[1], IMREAD_COLOR);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }

    image_width = image.cols;
    image_height = image.rows;
    warp_image = (uint8_t *)malloc(sizeof(uint8_t) * image_height * image_width * 3);

    // calculate transform matrix
    cal_trans_matrix(0.0, 0.0,10, image_width, image_height, trans_matrix);

    convert_trans_mat_to_fixed(trans_matrix, trans_matrix_fixed);

    
    start = clock();
    for (int i = 0; i < num_of_tests; i++)
    {
        warp_rgb_image(trans_matrix, image.data, image_width,
                       image_height, warp_image);
    }
    end = clock();
    elapsed = double(end - start) / CLOCKS_PER_SEC;
    printf("--------------------------\n");
    printf("Image warp in float operation:\n");
    printf("Time measured: %f seconds.\n", elapsed/num_of_tests);

    Mat img1(image_height, image_width, CV_8UC3, warp_image);
    imwrite("test_float.png", img1);

    start = clock();
    for (int i = 0; i < num_of_tests; i++)
    {
        warp_rgb_image_fixed(trans_matrix_fixed, image.data, image_width,
                             image_height, warp_image);
    }
    end = clock();
    elapsed = double(end - start) / CLOCKS_PER_SEC;
    printf("--------------------------\n");
    printf("Image warp in fixed-point operation:\n");
    printf("Time measured: %f seconds.\n", elapsed/num_of_tests);

    Mat img2(image_height, image_width, CV_8UC3, warp_image);
    imwrite("test_fixed.png", img2);
    
    free(warp_image);
    return 0;
}