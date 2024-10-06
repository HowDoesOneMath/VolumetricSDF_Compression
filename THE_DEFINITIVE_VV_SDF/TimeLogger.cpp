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

void TimeLogger::PrintTotalAndAverageAndGreatestTime(std::string key, std::string total_piece, std::string average_piece, std::string greatest_piece, std::string rest_of_messge)
{
	PrintLoggerTotalTime(key, total_piece + rest_of_messge);
	PrintLoggerAverageTime(key, average_piece + rest_of_messge);
	PrintLoggerGreatestTime(key, greatest_piece + rest_of_messge);
}

void TimeLogger::PrintEmptyLine()
{
	std::string to_print = "\n";
	main_file.write(to_print.c_str(), to_print.length());
}

void TimeLogger::PrintSolidLine(int length, char character)
{
	std::string to_print;

	to_print.resize(length, character);
	to_print += "\n";

	main_file.write(to_print.c_str(), to_print.length());
}

