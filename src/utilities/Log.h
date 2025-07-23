#ifndef LOG_H
#define LOG_H

#include <string>
#include <iostream>
#include <vector>
#include "../basic/Vertex.h"
#include <sstream>
#include <set>

namespace logger {
    // Log level enum
    enum class LogLevel {
        INFO,
        WARNING,
        ERR
    };

    // Log function
    inline void log(LogLevel level, const std::string& message) {
        switch (level) {
            case LogLevel::INFO:
                std::cout << "[INFO] " << message << std::endl;
                break;
            case LogLevel::WARNING:
                std::cout << "\033[33m[WARNING] " << message << "\033[0m" << std::endl;
                break;
            case LogLevel::ERR:
                std::cout << "\033[31m[ERROR] " << message << "\033[0m" << std::endl;
                break;
        }
    }

    // Convenient utility functions
    inline void log_info(const std::string& message) {
        log(LogLevel::INFO, message);
    }

    inline void log_warning(const std::string& message) {
        log(LogLevel::WARNING, message);
    }

    inline void log_error(const std::string& message) {
        log(LogLevel::ERR, message);
    }

    // Convert vector to string representation
    template<typename T>
    inline std::string vectorToString(const std::vector<T>& vec) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            ss << vec[i];
            if (i < vec.size() - 1) ss << ", ";
        }
        ss << "]";
        return ss.str();
    }

    // Specialization for Vertex
    template<>
    inline std::string vectorToString(const std::vector<Vertex>& vec) {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            ss << "(" << vec[i].x << "," << vec[i].y << ")";
            if (i < vec.size() - 1) ss << ", ";
        }
        ss << "]";
        return ss.str();
    }


    template<typename T>
    inline std::string setToString(const std::set<T>& set) {
        std::stringstream ss;
        ss << "[";
        size_t i = 0;
        for (const auto& item : set) {
            ss << item;
            if (i < set.size() - 1) ss << ", ";
            ++i;
        }
        ss << "]";
        return ss.str();
    }


}

#endif // LOG_H 