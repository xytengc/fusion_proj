#ifndef RUNTIME_LOG_H
#define RUNTIME_LOG_H

#include <string>

namespace runtime_log {

void setLogFile(const std::string& path);
void logLine(const std::string& message);
void logLineFileOnly(const std::string& message);

} // namespace runtime_log

#endif // RUNTIME_LOG_H
