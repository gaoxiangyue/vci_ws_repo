#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <mutex>

namespace ce {


enum class ELogType
{
    MESSAGE,
    WARNING,
    ERROR,
    DEBUG
};

//Thrad-safe std::cout logger
class Logger
{
public:
    ~Logger() {}

    virtual void log(ELogType type, const std::string &s);

protected:
    std::mutex mutex;
};


}

#endif // LOGGER_HPP

