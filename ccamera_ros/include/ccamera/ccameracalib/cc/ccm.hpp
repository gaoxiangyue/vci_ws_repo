#ifndef CCM_HPP
#define CCM_HPP

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <functional>

namespace ce {

/*
 * Interface "long playing" functions containing controller modules.
 * If some function:
 * - takes valuable time
 * And wants to:
 * - be interruptable
 * - run in a separate thread
 * Then:
 * - set mRunning = true as function begins
 * - call notify() before return
 * - setCallback() before actual function call(optionaly)
 * See examples:
 * - FrameGrabber
 * - CalibrationAlgorithm
 */
class CalibrationControllerModule
{
public:
    CalibrationControllerModule() :
        mCallback([]{}),
        mErrorCode(nullptr),
        mRunning(false),
        mInterrupt(false),
        mStopMutex(),
        mStopCondVar()
    {}

    bool running()
    {
        return mRunning;
    }

    void interrupt()
    {
        if(mRunning)
        {
            std::unique_lock<std::mutex> lk(mStopMutex);
            mInterrupt = true;
            while(mRunning)
                mStopCondVar.wait(lk);
            mInterrupt = false;
        }
    }

    void setCallback(std::function<void()> callback)
    {
        mCallback = callback;
    }

    void setErrorCodePtr(int* const ptr)
    {
        mErrorCode = ptr;
    }

    int errorCode()
    {
        if (mErrorCode)
            return *mErrorCode;
        else
            return 0;
    }

protected:
    void notify(int code)
    {
        if (mErrorCode)
            *mErrorCode = code;

        {
            std::lock_guard<std::mutex> lk(mStopMutex);
            mRunning = false;
        }
        mStopCondVar.notify_one();
        mCallback();
    }

    std::function<void()> mCallback;
    int* mErrorCode = nullptr;

    std::atomic<bool> mRunning;
    std::atomic<bool> mInterrupt;

    std::mutex mStopMutex;
    std::condition_variable mStopCondVar;
};


}

#endif // CCM_HPP

