#include "arm_nnfunctions.h"
#include "arm_nnsupportfunctions.h"
#include <assert.h>
/**
 *  @ingroup groupNN
 */
/**
 * @addtogroup NNConv
 * @{
 */
/**
 * @brief Q15-Q7 depthwise separable convolution function (non-square shape)
 * @param[in]       Im_in         pointer to input tensor
 * @param[in]       dim_im_in_x   input tensor dimension x
 * @param[in]       dim_im_in_y   input tensor dimension y
 * @param[in]       ch_im_in      number of input tensor channels
 * @param[in]       wt            pointer to kernel weights
 * @param[in]       ch_im_out     number of filters, i.e., output tensor channels
 * @param[in]       dim_kernel_x  filter kernel size x
 * @param[in]       dim_kernel_y  filter kernel size y
 * @param[in]       padding_x     padding sizes x
 * @param[in]       padding_y     padding sizes y
 * @param[in]       stride_x      convolution stride x
 * @param[in]       stride_y      convolution stride y
 * @param[in]       bias          pointer to bias
 * @param[in]       bias_shift    amount of left-shift for bias
 * @param[in]       out_shift     amount of right-shift for output
 * @param[in,out]   Im_out        pointer to output tensor
 * @param[in]       dim_im_out_x  output tensor dimension x
 * @param[in]       dim_im_out_y  output tensor dimension y
 * @param[in,out]   bufferA       pointer to buffer space for input
 * @param[in,out]   bufferB       pointer to buffer space for output
 * @return     The function returns either
 * <code>ARM_MATH_SIZE_MISMATCH</code> or <code>ARM_MATH_SUCCESS</code> based on the outcome of size checking.
 *
 * This function is the version with full list of optimization tricks, but with
 * some constraints:
 *   ch_im_in is equal to ch_im_out
 *
 */
arm_status arm_depthwise_separable_conv2x4_q15_q7_q31(const q15_t *Im_in,
                                                   const uint16_t dim_im_in_x,
                                                   const uint16_t dim_im_in_y,
                                                   const uint16_t ch_im_in,
                                                   const q7_t *wt,
                                                   const uint16_t ch_im_out,
                                                   const uint16_t dim_kernel_x,
                                                   const uint16_t dim_kernel_y,
                                                   const uint16_t padding_x,
                                                   const uint16_t padding_y,
                                                   const uint16_t stride_x,
                                                   const uint16_t stride_y,
                                                   const q31_t *bias,
                                                   const uint16_t out_shift,
                                                   q15_t *Im_out,
                                                   const uint16_t dim_im_out_x,
                                                   const uint16_t dim_im_out_y)
{
    assert(ch_im_in == ch_im_out);
    assert(padding_x == 1 && padding_y == 0);
    assert(stride_x == 2 && stride_y == 1);
    assert(dim_kernel_x == 4 && dim_im_in_y == 2);
    assert(dim_im_in_y == 2 && dim_im_out_y == 1);
    int element_per_w_ch = dim_kernel_x * dim_kernel_y;
    int element_per_im_h = dim_im_in_x * ch_im_in;
    //int element_per_out_ch = dim_im_out_x * dim_im_out_y;

    const q15_t *im_ptr = Im_in;
    const q7_t *wt_ptr = wt;
    q15_t *out_ptr = Im_out;
    q31_t sum;
    q31_t inM1, inM2;
    q31_t inV1, inV2;
    for (int i = 0; i < ch_im_in; i++) {
        q7_t wt_row1[] = {wt_ptr[0],wt_ptr[2],wt_ptr[1],wt_ptr[3]};
        q7_t wt_row2[] = {wt_ptr[4],wt_ptr[6],wt_ptr[5],wt_ptr[7]};
        const q7_t *wt_row1_ptr = wt_row1;
        const q7_t *wt_row2_ptr = wt_row2;
        const q15_t *input_data_row1 = im_ptr;
        const q15_t *input_data_row2 = im_ptr + element_per_im_h;
        sum = wt_ptr[1] * input_data_row1[0] + wt_ptr[5] * input_data_row2[0] + \
              wt_ptr[2] * input_data_row1[1] + wt_ptr[6] * input_data_row2[1] + \
              wt_ptr[3] * input_data_row1[2] + wt_ptr[7] * input_data_row2[2] + \
              bias[0] + NN_ROUND(out_shift);
              
        *out_ptr++ = (q15_t)(__SSAT((sum >> out_shift), 16));
        ++input_data_row1;
        ++input_data_row2;
        for (int j = 0; j < dim_im_out_x - 2; j++) {
            sum = ((q31_t)(bias[0])) + NN_ROUND(out_shift);
            inV1 = arm_nn_read_q15x2(input_data_row1);
            inV2 = arm_nn_read_q15x2(input_data_row1+2);
            inM1 = arm_nn_read_q7x4(wt_row1_ptr);
            inM2 = __SXTB16(__ROR(inM1, 8));
            inM1 = __SXTB16(inM1);
            sum = __SMLAD(inM1, inV1, sum);
            sum = __SMLAD(inM2, inV2, sum);
            inV1 = arm_nn_read_q15x2(input_data_row2);
            inV2 = arm_nn_read_q15x2(input_data_row2+2);
            inM1 = arm_nn_read_q7x4(wt_row2_ptr);
            inM2 = __SXTB16(__ROR(inM1, 8));
            inM1 = __SXTB16(inM1);
            sum = __SMLAD(inM1, inV1, sum);
            sum = __SMLAD(inM2, inV2, sum);
            *out_ptr++ = (q15_t)(__SSAT((sum >> out_shift), 16));
            input_data_row1 += 2;
            input_data_row2 += 2;
        }
        sum = wt_ptr[0] * input_data_row1[0] + wt_ptr[4] * input_data_row2[0] + \
                     wt_ptr[1] * input_data_row1[1] + wt_ptr[5] * input_data_row2[1] + \
                     wt_ptr[2] * input_data_row1[2] + wt_ptr[6] * input_data_row2[2] + \
                     bias[0] + NN_ROUND(out_shift);
        *out_ptr++ = (q15_t)(__SSAT((sum >> out_shift), 16));
        wt_ptr += element_per_w_ch;
        im_ptr += dim_im_in_x;
        ++bias;
    }
    return (ARM_MATH_SUCCESS);
}
