/**
 *******************************************************************************
 *
 * @file ccamera-pixel-format.h
 * @brief Cogent Camera API supported pixel formats.
 * @version 2.3.0
 * @date 28-03-2018
 * @copyright 2016-2017 Cogent Embedded Inc.
 *
 * THIS FILE IS SUBJECT TO THE LICENSE TERMS PROVIDED IN THE 'LICENSE' FILE FOUND
 * IN THE TOP-LEVEL DIRECTORY OF THIS SOFTWARE PACKAGE OR BY REQUEST VIA
 * http://cogentembedded.com, source@cogentembedded.com
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE.
 *******************************************************************************/ 

#ifndef CCAMERA_PIXEL_FORMAT_H
#define CCAMERA_PIXEL_FORMAT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Supported pixel formats.
 * All pixel formats have memory layout according to FOURCC standart.
 * To extract pixel data use size, offset and stride fields, provided in @camera_buf_t structure.
 * The number of supported pixel formats may be extended in external headers.
 */
typedef unsigned int CameraPixelFormat;

/**
 * @brief Compressed JPEG data.
 */
#define CCAMERA_FORMAT_JPEG  0x0001

/**
 * @brief YUV 4:2:0. 8 bit Y plane followed by 8 bit 2x2 subsampled U and V planes.
 */
#define CCAMERA_FORMAT_I420  0x0002

/**
 * @brief 24bpp RGB format.
 */
#define CCAMERA_FORMAT_RGB   0x0003

/**
 * @brief Same as CCAMERA_FORMAT_RGB, but with different component order.
 */
#define CCAMERA_FORMAT_BGR   0x0004

/**
 * @brief 32bpp BGR format, last byte filled with zeroes.
 */
#define CCAMERA_FORMAT_BGRX  0x0005

/**
 * @brief 32bpp RGB format, first component filled with zeroes.
 */
#define CCAMERA_FORMAT_XRGB  0x0006

/**
 * @brief 32bpp RGBA format, last component is alpha channel.
 */
#define CCAMERA_FORMAT_RGBA  0x0007

/**
 * @brief 32bpp ARGB format, first component is alpha channel.
 */
#define CCAMERA_FORMAT_ARGB  0x0008

/**
 * @brief YUV 4:2:2 with YUYV pixel order.
 */
#define CCAMERA_FORMAT_YUY2  0x0009

/**
 * @brief YUV 4:2:0. 8 bit Y plane followed by 8 bit 2x2 subsampled V and U planes.
 */
#define CCAMERA_FORMAT_YV12  0x000A

/**
 * @brief YUV 4:2:2. Y sample at every pixel, U and V sampled at every second pixel horizontally on each line.
 */
#define CCAMERA_FORMAT_UYVY  0x000B

/**
 * @brief YUV 4:2:2. 8-bit Y plane followed by an interleaved U/V plane with 2x1 subsampling.
 */
#define CCAMERA_FORMAT_NV16  0x000C

/**
 * @brief YUV 4:2:0. 8-bit Y plane followed by an interleaved U/V plane with 2x2 subsampling.
 */
#define CCAMERA_FORMAT_NV12  0x000D

/**
 * @brief Single plane, containing 8bpp grayscale image.
 */
#define CCAMERA_FORMAT_GRAY8 0x000E

/**
 * @brief YUV 4:2:0. 8 bit Y plane followed by 8 bit 2x2 subsampled U and V planes.
 */
#define CCAMERA_FORMAT_I420_10LE  0x000F

#ifdef __cplusplus
}
#endif

#endif /* CCAMERA_PIXEL_FORMAT_H */

