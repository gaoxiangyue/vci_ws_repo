/**
 *******************************************************************************
 *
 * @file ccamera.h
 * @brief Cogent Camera API.
 * @version 2.3.0
 * @date 28-03-2018
 * @copyright 2015-2017 Cogent Embedded Inc.
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

#ifndef CCAMERA_H
#define CCAMERA_H

#include <stddef.h>
#include <ccamera/ccamera-pixel-format.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The maximum number of planes supported in the API.
 */
#define CCAMERA_MAX_PLANES 4

/**
 * @brief  Supported memory types.
 * The number of supported memory types may be extended in external headers.
 */    
typedef unsigned int CameraMemoryType;
   
/**
 * @brief Default memory type, accessible by CPU directly.
 */
#define CAMERA_MEMORY_TYPE_SYSTEM 0x0001

/**
 * @brief Single plane structure.
 */
typedef struct camera_plane {
    /**
     * @brief Generic pointer to camera data for all planes.
     * May contain raw image data or OpenGL/OpenCL id for the given plane.
     * CUDA device pointer may be used for each plane.
     * @note This field is read-only and must NOT be written, freed or changed in any other means.
     */
    const void* data;

    /**
     * @brief The size of data for each plane in bytes.
     */
    size_t size;

    /**
     * @brief Offset to the data in each plane relative to it's data.
     */
    size_t offset;

    /**
     * @brief Stride of each plane.
     * @note Absolute value of the stride may be greater than the image width due to padding.
     */
    int stride;
} camera_plane_t;

/**
 * @brief Camera buffer as returned by camera_get_buffer()
 */
typedef struct camera_buf {
    /**
     * @brief Camera handle as returned by camera_open() or camera_open_with_args().
     * @note This handle is a file descriptor on Linux and Android.
     */
    int camera_fd;

    union {
        camera_plane_t planes[CCAMERA_MAX_PLANES];

        struct
        {
            /**
             * @brief Generic pointer to camera data for all planes.
             * May contain raw image data or OpenGL/OpenCL id for the given plane.
             * CUDA device pointer may be used for each plane.
             * @note This field is read-only and must NOT be written, freed or changed in any other means.
             */
            const void* data;

            /**
             * @brief The size of data for each plane in bytes.
             */
            size_t size;

            /**
             * @brief Offset to the data in each plane relative to it's data.
             */
            size_t offset;

            /**
             * @brief Stride of each plane.
             * @note Absolute value of the stride may be greater than the image width due to padding.
             */
            int stride;
        };
    };

    /**
     * @brief Buffer format in gstreamer-1.0 format.
     * @note This field is read-only and must NOT be written, freed or changed in any other means.
     */
    const char* caps;

    /**
     * @brief Buffer timestamp.
     * @note In nanoseconds.
     */
    unsigned long int timestamp;
    
    /**
     * @brief Image width.
     */
    int width;

    /**
     * @brief Image height.
     */
    int height;
    
    /**
     * @brief Number of planes.
     */
    int n_planes;
    
    /**
     * @brief Camera buffer memory type. See @CameraMemoryType for more details.
     */
    CameraMemoryType memory_type;
    
    /**
     * @brief Camera buffer pixel format. See @CameraPixelFormat for more details.
     */
    CameraPixelFormat format;

    /**
     * @brief Reserved for internal use.
     */
    int padding[3];
} camera_buf_t;

/**
 * Typedef for user-provided callback.
 * The callback is called when new buffer is received on the client side.
 * @param camera_fd Camera Id returned by camera_open() or camera_open_with_args().
 * @param buf Pointer to @ref camera_buf_t structure
 * @param context User-provided context;
 */
typedef void(*camera_newbuffer_callback)(int camera_fd, camera_buf_t* buf, void* context);

/**
 * Open the camera in a specific buffer format.
 * Camera data is shared between different processes
 * To preserve original aspect ratio when resizing image (letterboxing) add "pixel-aspect-ratio=1/1" to caps string.
 * To use DMA memory add "memory=dma" to caps string.
 * @param id Camera id (MAC address). This field is case-insensitive
 * @param caps Video capabilites in gstreamer-1.0 format
 * @param flags Additional flags.
 *        O_NONBLOCK defined in <fcntl.h> makes camera_get_buffer() and camera_get_otp_id() calls non-blocking.
 * @returns Camera descriptor
 * @retval 0 or negative if an error occurred. See errno() for more details.
 */
int camera_open(const char *id, const char *caps, int flags);

/**
 * This is an example how to use camera_open() function.
 * @example camera_open.c
 */

/**
 * Same as camera_open() but allows to specify additional parameters: image format, resolution and frame rate.
 * Supported buffer types are: I420, RGB, BGR, ARGB, BGRx, JPEG.
 * To preserve original aspect ratio when resizing image (letterboxing) add "pixel-aspect-ratio=1/1" to caps string.
 * To use DMA memory add "memory=dma" to caps string.
 * To open camera in 'native' format do not specify width, height, format and framerate - set this parameters to 0 (NULL).
 * @param id Camera id (MAC address). This field is case-insensitive
 * @param caps Video capabilites in gstreamer-1.0 format. Default: NULL
 * @param flags Additional flags @see camera_open()
 * @param buffer_type Buffer type ("I420", "RGB", "BGR", "ARGB", "BGRx" ,"JPEG")
 * @param width Image width in pixels
 * @param height Image height in pixels
 * @param framerate Frame rate in Hz
 * @returns Camera descriptor
 * @retval 0 or negative if an error occurred. See errno() for more details.
 */
int camera_open_with_args(const char *id, const char* caps, int flags,
						  const char *buffer_type, int width, int height, float framerate);

/**
 * This is an example how to use camera_open_with_args() function.
 * @example camera_open_with_args.c
 */


/**
 * Register a callback for PUSH mode: the server pushes a buffer to the client.
 * @param camera_fd Camera Id returned by camera_open() or camera_open_with_args().
 * @param cb Callback function
 * @param context User-provided context;
 * @retval 0 if success
 * @retval -ENOENT if camera_fd not found
 */
int camera_register_callback(int camera_fd, camera_newbuffer_callback cb, void *context);

/**
 * The function works in PULL mode: the client requests the buffer.
 * When the buffer is not needed anymore, the client must call camera_release_buffer().
 * The function is thread-safe and can be called from multiple threads.
 * If flag O_NONBLOCK was set for the camera, this call does not block,
 * and immediately returns latest available buffer.
 * @param camera_fd Camera Id returned by camera_open() or camera_open_with_args().
 * @param flags Additional flags @see camera_open()
 * @returns Data to @ref camera_buf_t structure
 * @retval NULL if an error occurred. See errno() for more details.
 */
camera_buf_t* camera_get_buffer(int camera_fd, int flags);

/**
 * This is an example how to use camera_get_buffer() function.
 * @example camera_get_buffer.c
 */

/**
 * Release the buffer when it is no longer needed.
 * @param buffer Camera buffer as returned by camera_get_buffer().
 */
void camera_release_buffer(camera_buf_t* buffer);

/**
 * @brief Get camera OTP ID. Returned string is formated as "%02x-%02x-%02x-%02x-%02x-%02x".
 * @note Returned value is allocated by Camera API and becomes invalid after camera_close().
 * If OTP ID is not available for the camera, returned value is NULL and errno is set to -ENOTSUP.
 * If O_NONBLOCK is set for the camera this call does not block and
 * -EAGAIN is set if request is pending.
 * Call camera_get_error() for more details about the error.
 * @param camera_fd
 * @return null-terminated string or NULL on error.
 */
const char* camera_get_otp_id(int camera_fd);

/**
 * Close the camera and release all resources. Releases all buffers returned by camera_get_buffer() making them unusable.
 * @param camera_fd
 * @retval Negative if the error occurred. See errno() for more details.
 */
int camera_close(int camera_fd);

/**
 * Get error and description of latest Camera API call.
 * @param camera_fd
 * @return 
 */
int camera_get_error(int camera_fd, char** error_message);

/**
 * @brief Free error message set by camera_get_error().
 */
void camera_free_error_message(char* error_message);

/**
 * Enable logging and set log_level logging.
 * @note This function must be invoked before any other Camera API call,
 * otherwise it will not take effect and log level will be set according to CAMERA_API_LOG_LEVEL environment variable.
 * @param log_level If NULL log level INFO is set by default.
 * @c{TRACE, DEBUG, INFO, MESSAGE, WARNING, CRITICAL, ERROR} .
 * 
 */
void camera_api_set_log_level(const char* log_level);

#ifdef __cplusplus
}
#endif

#endif // CCAMERA_H
