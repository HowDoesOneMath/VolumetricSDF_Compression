#include "TimeLogger.h"

void TimeLogger::StartLogging(std::string logger_name)
{
	main_file.open(logger_name.c_str());
}

void TimeLogger::CloseLoggers()
{
	if (main_file.is_open())
	{
		main_file.close();
	}

	for (auto& logger : loggers)
	{
		delete logger.second;
	}

	loggers.clear();
}

void TimeLogger::CreateNewLogger(std::string key, IndependentLogger *new_logger)
{
	loggers[key] = new_logger;
}

TimeLogger::IndependentLogger* TimeLogger::GetLogger(std::string key)
{
	return loggers[key];
}

void TimeLogger::PrintLogger(std::string key, std::string message)
{
	std::string to_print = loggers[key]->GetName() + message + std::to_string(loggers[key]->GetTime() * nanoseconds_to_seconds) + "\n";
	main_file.write(to_print.c_str(), to_print.length());
}

void TimeLogger::PrintLoggerTotalTime(std::string key, std::string message)
{
	std::string to_print = loggers[key]->GetName() + message + std::to_string(loggers[key]->GetTotalTime() * nanoseconds_to_seconds) + "\n";
	main_file.write(to_print.c_str(), to_print.length());
}

void TimeLogger::PrintLoggerGreatestTime(std::string key, std::string message)
{
	std::string to_print = loggers[key]->GetName() + message + std::to_string(loggers[key]->GetGreatestTime() * nanoseconds_to_seconds) + "\n";
	main_file.write(to_print.c_str(), to_print.length());
}

void TimeLogger::PrintLoggerAverageTime(std::string key, std::string message)
{
	std::string to_print = loggers[key]->GetName() + message + std::to_string((loggers[key]->GetTotalTime() * nanoseconds_to_seconds) / loggers[key]->GetTotalStamps()) + "\n";
	main_file.write(to_print.c_str(), to_print.length());
}

void TimeLogger::PrintEmptyLine()
{
	std::string to_print = "\n";
	main_file.write(to_print.c_str(), to_print.length());
}

