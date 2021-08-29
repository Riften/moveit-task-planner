/**
 * @author Riften
 */

#ifndef RFPDDL_ROBOTFLOW_LOG_H
#define RFPDDL_ROBOTFLOW_LOG_H

#include <string>
#include <log4cxx/logger.h>
#include <boost/filesystem.hpp>
#include <mutex>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/basicconfigurator.h>
#include <iostream>

/// Define a logger
/// Deprecated: Use RF_STATIC_LOGGER instead.
#define RF_LOGGER_DEF(NAME) robotflow::Log4cxxWrapper robotflow_logger_ = robotflow::Log4cxxWrapper(#NAME);
#define RF_STATIC_LOGGER(VAR, NAME) static robotflow::Log4cxxWrapper VAR = robotflow::Log4cxxWrapper(#NAME);
// #define RF_STATIC_LOGGER(NAME) static robotflow::Log4cxxWrapper robotflow_logger_ = robotflow::Log4cxxWrapper(#NAME);

#define RF_DEBUG(MSG) LOG4CXX_DEBUG(robotflow_logger_, MSG);
#define RF_ERROR(MSG) LOG4CXX_ERROR(robotflow_logger_, MSG);
#define RF_INFO(MSG) LOG4CXX_INFO(robotflow_logger_, MSG);

namespace robotflow {
    static inline bool file_exists(const std::string& file_path) {
        std::ifstream f(file_path.c_str());
        return f.good();
    }

    /**
     * @note initLogSystem must be static otherwise there would be a multi-def error.
     */
    void inline initLogSystem() {
        boost::filesystem::path DEFAULT_LOG4CXX_CONFIG_PATH =
                boost::filesystem::path(std::getenv("HOME")) / boost::filesystem::path(".rfpddl.config");
        if(file_exists(DEFAULT_LOG4CXX_CONFIG_PATH.string())) {
            std::cout << "... Initialize log system from " << DEFAULT_LOG4CXX_CONFIG_PATH << std::endl;
            log4cxx::PropertyConfigurator::configure(DEFAULT_LOG4CXX_CONFIG_PATH.string());
        } else {
            std::cout << "... Initialize default log system." << std::endl;
            log4cxx::BasicConfigurator::configure();
        }
    }

    /**
     * Log4cxxWrapper is a wrapper of log4cxx::LoggerPtr
     *
     * @details The main reason we wrap log4cxx::LoggerPtr into a new class is
     * to initialize log4cxx system during the creation of wrapper.
     * Besides, Wrapper class also makes it easier to control the initialization
     * behavior within a multi-threading environment.
     *
     * @todo Make Log4cxxWrapper suitable for multi-threading environment.
     *
     * @note There is no default constructor for Log4cxxWrapper.
     * That is in order to avoid definition of empty logger.
     */
    class Log4cxxWrapper {
    public:
        explicit Log4cxxWrapper(const std::string& name) {
            /**
             * static variable need explicit definition, which makes it difficult to
             * use static member variable in header only library.
             * In that case, we use a function local static variable instead.
             *
             * Other solutions can be found in
             * https://stackoverflow.com/questions/11709859/how-to-have-static-data-members-in-a-header-only-library
             */
            static std::once_flag initialize_flag_;
            std::call_once(initialize_flag_, initLogSystem);
            logger_ = log4cxx::Logger::getLogger(name);
        };

        void inline debug(const std::string& str){
            logger_->debug(str);
        };

        void inline info(const std::string& str) {
            logger_->info(str);
        };

        void inline warn(const std::string& str) {
            logger_->warn(str);
        };

        void inline error(const std::string& str) {
            logger_->error(str);
        };

        inline log4cxx::LoggerPtr& logger() {
            return logger_;
        }

        operator log4cxx::LoggerPtr() {
            return logger_;
        }
    private:

        // static std::once_flag initialize_flag_;
        log4cxx::LoggerPtr logger_;
    };

}

#endif //RFPDDL_ROBOTFLOW_LOG_H
