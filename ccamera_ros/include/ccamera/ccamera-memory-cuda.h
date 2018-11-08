/**
 *******************************************************************************
 *
 * @file ccamera-memory-cuda.h
 * @brief Cogent Camera API CUDA memory support.
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

#ifndef CCAMERA_CUDA_H
#define CCAMERA_CUDA_H

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_MEMORY_TYPE_CUDA 0x0003

/**
 * Check whether buffer contains DMA fd memory.
 * @param buffer
 * @retval 1 if TRUE
 * @retval 0 if FALSE
 */
#define camera_buffer_is_cuda(buffer) (buffer->memory_type == CAMERA_MEMORY_TYPE_CUDA)

#ifdef __cplusplus
}
#endif

#endif /* CCAMERA_CUDA_H */

