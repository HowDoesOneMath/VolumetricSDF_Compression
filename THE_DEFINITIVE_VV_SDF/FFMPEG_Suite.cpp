#include "FFMPEG_Suite.h"

#include "AdditionalUtilities.h"

void FFMPEG_Suite::FillImageWithTestPattern(int offset, int modulo_value, int r_thresh, int g_thresh, int b_thresh)
{
	int arbitrary_modulo;

	for (int w = 0; w < single_frame.width(); ++w)
	{
		for (int h = 0; h < single_frame.height(); ++h)
		{
			arbitrary_modulo = (h + w + offset) % modulo_value;

			single_frame(w, h, 0, 0) = (arbitrary_modulo > r_thresh) ? 255 : 0;
			single_frame(w, h, 0, 1) = (arbitrary_modulo > g_thresh) ? 255 : 0;
			single_frame(w, h, 0, 2) = (arbitrary_modulo > b_thresh) ? 255 : 0;
		}
	}

	//single_frame = single_frame.RGBtoYUV();
}

void FFMPEG_Suite::SaveArbitraryVideo()
{
	int requested_frame_count = 20;

	FFMPEG_WrapperStreamParams new_params;

	new_params.h = 1080;
	new_params.w = 1920;
	new_params.format = AVPixelFormat::AV_PIX_FMT_YUV420P;
	new_params.frames_per_second = { 4, 1 };
	new_params.encoder_choice = AVCodecID::AV_CODEC_ID_H264;
	new_params.expected_stream_length = requested_frame_count;
	new_params.compression_speed = FFMPEG_WRAPPER_COMPRESSION_SPEED::ULTRA_FAST;

	single_frame.assign(new_params.w, new_params.h, 1, 3, 0);


	if (!output_wrapper.OpenVideoFileForWriting(new_video_file, new_params))
	{
		return;
	}

	for (int i = 0; i < requested_frame_count; ++i)
	{
		FillImageWithTestPattern(i * 8, 200, 40, 80, 120);

		if (!output_wrapper.SaveFrame(single_frame))
		{
			return;
		}
	}

	output_wrapper.CloseVideoFile();
}

void FFMPEG_Suite::LoadArbitraryVideo()
{
	std::string image_save_name;
	int iter;

	iter = 0;
	input_wrapper.OpenVideoFileForReading(new_video_file);
	while (input_wrapper.RetrieveFrame(single_frame))
	{
		image_save_name = file_dump_from_code + GetNumberFixedLength(iter, 6) + singular_image_ext;
		single_frame.save_png(image_save_name.c_str());
		++iter;
	}
	input_wrapper.CloseVideoFile();

	iter = 0;
	input_wrapper.OpenVideoFileForReading(video_to_load);
	while (input_wrapper.RetrieveFrame(single_frame))
	{
		image_save_name = file_dump_command_line + GetNumberFixedLength(iter, 6) + singular_image_ext;
		single_frame.save_png(image_save_name.c_str());
		++iter;
	}
	input_wrapper.CloseVideoFile();
}

void FFMPEG_Suite::run(int argc, char** argv)
{
	SaveArbitraryVideo();

	//LoadArbitraryVideo();
}
