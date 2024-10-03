#pragma once

#include <string>
#include <vector>
#include <queue>
#include <iostream>
#include <CImg.h>

//REMEMBER! FFMPEG needed to have certain linker dependencies set to Windows SDKs!

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/avutil.h>
#include <libswscale/swscale.h>
}

enum FFMPEG_WRAPPER_STATE
{
	CLOSED,
	OPEN_READING,
	OPEN_WRITING
};

enum FFMPEG_WRAPPER_WRITING_STATE
{
	WRITING_ERROR,
	MORE_INPUT,
	REACHED_EOF,
	WRITTEN
};

enum FFMPEG_WRAPPER_COMPRESSION_SPEED
{
	ULTRA_FAST,
	SUPER_FAST,
	VERY_FAST,
	FASTER,
	FAST,
	MEDIUM,
	SLOW,
	SLOWER,
	VERY_SLOW,
	//Do not use this setting
	PLACEBO
};

struct FFMPEG_WrapperStreamParams
{
	int w, h;
	int frame_group_size = 10;
	int max_allowed_b_frames = 0;
	AVPixelFormat format;
	FFMPEG_WRAPPER_COMPRESSION_SPEED compression_speed = FFMPEG_WRAPPER_COMPRESSION_SPEED::MEDIUM;
	AVCodecID encoder_choice;
	AVRational frames_per_second;
	size_t expected_stream_length = 0;
};

class FFMPEG_Wrapper
{
	int result;

	//const int max_streams = 16;

	size_t current_frame = 0;
	size_t current_packet = 0;

	AVFormatContext* format_ctx = nullptr;
	AVCodecContext* codec_ctx = nullptr;
	AVStream* reading_stream = nullptr;
	AVPacket* packet = nullptr;
	AVFrame* rgb_frame = nullptr;
	AVFrame* other_format_frame = nullptr;

	FFMPEG_WRAPPER_STATE state;

	AVPixelFormat pix_fmt = AVPixelFormat::AV_PIX_FMT_NONE;

	SwsContext* rgb_converter_ctx = nullptr;

	void IterateOverDictionary(AVDictionary* dictionary);

	std::string GetErrorString(int error_code);

	std::string GetCompressionSpeedAsString(FFMPEG_WRAPPER_COMPRESSION_SPEED speed);

	bool InitializeFrameInterop(int w, int h, AVPixelFormat format, int flags, bool is_writing);
	void CleanupFrameInterop();

	void RawCopyToRGB(AVFrame* dst, cimg_library::CImg<unsigned char>& src);
	void RawCopyToCImg(cimg_library::CImg<unsigned char>& dst, AVFrame* src);
	bool EncodeFrame(AVFrame* input_frame);
	bool DecodeFrame(AVFrame* output_frame);

	bool OpenReadingFile(std::string &filename);
	bool GetReadingStreams();

	bool OpenWritingFile(std::string& filename, FFMPEG_WrapperStreamParams& params);
	bool CreateWritingStream(FFMPEG_WrapperStreamParams &params);
	bool WriteHeader(std::string& filename);

	void CloseReadingStreams();
	void CloseReadingFile();

	void WriteTrailer();
	void CloseWritingStream();
	void CloseWritingFile();

	bool CheckNotOpen();

	//double GetSSIM(cimg_library::CImg<unsigned char>& img1, cimg_library::CImg<unsigned char>& img2);

public:
	FFMPEG_Wrapper();
	~FFMPEG_Wrapper();

	bool OpenVideoFileForReading(std::string filename);
	bool OpenVideoFileForWriting(std::string filename, FFMPEG_WrapperStreamParams &params);

	bool SaveFrame(cimg_library::CImg<unsigned char> &to_save);
	bool RetrieveFrame(cimg_library::CImg<unsigned char>& to_load);

	void CloseVideoFile();

	FFMPEG_WRAPPER_STATE GetState() { return state; }
};