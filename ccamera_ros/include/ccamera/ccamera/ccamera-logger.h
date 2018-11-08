/**
 *******************************************************************************
 *
 * @file ccamera-logger.h
 * @brief Cogent Camera API logging facility.
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

#ifndef CAMERA_LOGGER_H
#define CAMERA_LOGGER_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct _CameraLogger CameraLogger;

/**
 * Create new CameraLogger instance
 * @param domain
 * @return new CameraLogger instance.
 */
CameraLogger* camera_logger_new(const char* domain);

/**
 * Free CameraLogger instance
 * @param logger
 */
void camera_logger_free(CameraLogger* logger);

/**
 * Add log message to the logger queue. Event timestamp is generated at the moment when the message was logged.
 * @param logger
 * @param format
 * @param ...
 */
void camera_logger_log(CameraLogger* logger, const char *format, ...);

/**
 * Add log message to the logger queue. Event timestamp is provided by user.
 * Use @camera_logger_get_timestamp() to capture event timestamp. 
 * @param logger
 * @param event_timestamp
 * @param format
 * @param ...
 */
void camera_logger_log_with_timestamp(CameraLogger* logger, unsigned long long event_timestamp, const char *format, ...);

/**
 * @return current CLOCK_MONOTONIC time in microseconds
 */
unsigned long long camera_logger_get_timestamp(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_LOGGER_H */

