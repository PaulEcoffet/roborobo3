/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "Utilities/LogManager.h"
#include "RoboroboMain/roborobo.h"


LogManager::LogManager(std::string __logFilename)
{
    logFile.open(__logFilename);
    if(!logFile) {
        std::cout << "[CRITICAL] Cannot open log file " << __logFilename << "." << std::endl << std::endl;
        std::cout << "Error: " << strerror(errno) << std::endl;
        exit(-1);
    }

}

LogManager::LogManager()
{
    std::cout << "[CRITICAL] Missing file name in LogManager(). Exiting." << std::endl << std::endl;
    exit(-1);
}

LogManager::~LogManager()
{
    logFile.close();
}

void LogManager::write(std::string str)
{
    logFile << str;
}

void LogManager::flush()
{
    logFile.flush();
}
