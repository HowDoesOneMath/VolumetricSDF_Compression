#pragma once

#include <fstream>
#include <chrono>
#include <iostream>

#include <unordered_map>
#include <string>

class TimeLogger
{
public:
	class IndependentLogger
	{
		std::chrono::high_resolution_clock::time_point tp_now;
		size_t nanosecond_count = 0;

		std::string logger_name;

		size_t total_nanoseconds = 0;
		size_t total_stamps = 0;

		size_t maximum_nanoseconds = 0;

	public:
		IndependentLogger(std::string name) {
			logger_name = name;
		}

		void StartTimer() {
			tp_now = std::chrono::high_resolution_clock::now();
		}

		void MarkTime() {
			nanosecond_count = (std::chrono::high_resolution_clock::now() - tp_now).count();
			total_nanoseconds += nanosecond_count;

			bool is_greater = nanosecond_count > maximum_nanoseconds;
			maximum_nanoseconds = maximum_nanoseconds * !is_greater + nanosecond_count * is_greater;

			++total_stamps;
		}

		void MarkCustomTime(size_t custom_time)
		{
			nanosecond_count = custom_time;
			total_nanoseconds += nanosecond_count;

			bool is_greater = nanosecond_count > maximum_nanoseconds;
			maximum_nanoseconds = maximum_nanoseconds * !is_greater + nanosecond_count * is_greater;

			++total_stamps;
		}

		void SetTime(size_t new_time) {
			nanosecond_count = new_time;
		}

		void ResetTotalTime() {
			total_nanoseconds = 0;
			maximum_nanoseconds = 0;
			total_stamps = 0;
		}

		void AddTime(size_t to_add) {
			nanosecond_count += to_add;
			total_nanoseconds += to_add;

			bool is_greater = nanosecond_count > maximum_nanoseconds;
			maximum_nanoseconds = maximum_nanoseconds * !is_greater + nanosecond_count * is_greater;
		}

		size_t GetTime() {
			return nanosecond_count;
		}

		size_t GetTotalTime() {
			return total_nanoseconds;
		}

		size_t GetGreatestTime() {
			return maximum_nanoseconds;
		}

		size_t GetTotalStamps() {
			return total_stamps;
		}

		std::string GetName() {
			return logger_name;
		}
	};

private:
	std::ofstream main_file;

	std::chrono::high_resolution_clock::time_point tp_now;
	size_t nanosecond_count;

	int tab_count;

	std::unordered_map<std::string, IndependentLogger*> loggers;

	double nanoseconds_to_seconds = 0.000000001;

public:
	void StartLogging(std::string logger_name);

	void CloseLoggers();

	void CreateNewLogger(std::string key, IndependentLogger *new_logger);

	IndependentLogger* GetLogger(std::string key);

	void PrintLogger(std::string key, std::string message);

	void PrintLoggerTotalTime(std::string key, std::string message);
	void PrintLoggerGreatestTime(std::string key, std::string message);
	void PrintLoggerAverageTime(std::string key, std::string message);

	void PrintTotalAndAverageAndGreatestTime(std::string key, 
		std::string total_piece, std::string average_piece, std::string greatest_piece, std::string rest_of_messge);

	void PrintEmptyLine();
	
	void PrintSolidLine(int length, char character);
};