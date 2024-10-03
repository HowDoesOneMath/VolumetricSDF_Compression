#pragma once

#include <string>
#include <iostream>
#include <wavelib.h>

class WaveletEncoder
{
	//NOT FINISHED, TODO

	std::string wavelet_type = "dwt";

public:
	void SetWaveletType(std::string new_type) { wavelet_type = new_type; }

	template<typename T>
	void Encode(std::vector<T> &input_signal, std::vector<T> &output_signal, int decomposition_count);

	template<typename T>
	void Decode(std::vector<T>& input_signal, std::vector<T>& output_signal, int decomposition_count);
};

template<typename T>
inline void WaveletEncoder::Encode(std::vector<T>& input_signal, std::vector<T>& output_signal, int decomposition_count)
{
	wave_object obj;
	wt_object wt;

	const char* name = "db4";
	obj = wave_init(name);

	wt = wt_init(obj, wavelet_type.c_str(), input_signal.size(), decomposition_count);
	setWTConv(wt, "direct");

	dwt(wt, input_signal.data());

	output_signal.resize(wt->outlength);
	
	for (int i = 0; i < wt->outlength; ++i) {
		output_signal[i] = wt->output[i];
	}

	wave_free(obj);
	wt_free(wt);
}
