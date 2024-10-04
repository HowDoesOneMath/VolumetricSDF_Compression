#include "FFMPEG_Wrapper.h"

void FFMPEG_Wrapper::IterateOverDictionary(AVDictionary* dictionary)
{
    std::cout << "Dictionary entries: " << av_dict_count(dictionary) << std::endl;

    const AVDictionaryEntry* entry = nullptr;
    entry = av_dict_iterate(dictionary, entry);
    while (entry != nullptr)
    {
        std::cout << entry->key << ", " << entry->value << std::endl;
        entry = av_dict_iterate(dictionary, entry);
    }
}

std::string FFMPEG_Wrapper::GetErrorString(int error_code)
{
    std::string to_return;
    to_return.resize(64);

    int error_result = av_strerror(error_code, to_return.data(), to_return.size());
    if (error_result < 0)
    {
        std::cout << "Failed to get error of error" << std::endl;
        return "";
    }

    return to_return;
}

std::string FFMPEG_Wrapper::GetCompressionSpeedAsString(FFMPEG_WRAPPER_COMPRESSION_SPEED speed)
{
    switch (speed)
    {
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::ULTRA_FAST:
        return "ultrafast";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::SUPER_FAST:
        return "superfast";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::VERY_FAST:
        return "veryfast";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::FASTER:
        return "faster";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::FAST:
        return "fast";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::MEDIUM:
        return "medium";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::SLOW:
        return "slow";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::SLOWER:
        return "slower";
    case FFMPEG_WRAPPER_COMPRESSION_SPEED::VERY_SLOW:
        return "veryslow";
    }

    std::cout << "Could not find compression speed that matched " << speed << std::endl;
    return "veryslow";
}

bool FFMPEG_Wrapper::InitializeFrameInterop(int w, int h, AVPixelFormat format, int flags, bool is_writing)
{
    pix_fmt = format;

    other_format_frame = av_frame_alloc();

    if (is_writing)
    {
        other_format_frame->width = w;
        other_format_frame->height = h;
        other_format_frame->format = pix_fmt;

        result = av_frame_get_buffer(other_format_frame, 0);
        if (result < 0) {
            std::cout << "Could not create video frame: " << result << ", " << GetErrorString(result) << std::endl;
            return false;
        }
    }


    rgb_frame = av_frame_alloc();

    if (is_writing)
    {
        rgb_frame->width = w;
        rgb_frame->height = h;
        rgb_frame->format = AVPixelFormat::AV_PIX_FMT_RGB24;

        result = av_frame_get_buffer(rgb_frame, 0);
        if (result < 0) {
            std::cout << "Could not create RGB frame: " << result << ", " << GetErrorString(result) << std::endl;
            return false;
        }
    }

    packet = av_packet_alloc();


    if (is_writing)
    {
        rgb_converter_ctx = sws_getContext(
            w, h, AVPixelFormat::AV_PIX_FMT_RGB24,
            w, h, pix_fmt,
            flags, nullptr, nullptr, nullptr);
    }
    else
    {
        rgb_converter_ctx = sws_getContext(
            w, h, pix_fmt,
            w, h, AVPixelFormat::AV_PIX_FMT_RGB24,
            flags, nullptr, nullptr, nullptr);
    }

    return true;
}

void FFMPEG_Wrapper::CleanupFrameInterop()
{
    if (other_format_frame)
    {
        av_frame_free(&other_format_frame);
    }

    if (rgb_frame)
    {
        av_frame_free(&rgb_frame);
    }

    if (packet)
    {
        av_packet_free(&packet);
    }

    if (rgb_converter_ctx)
    {
        sws_freeContext(rgb_converter_ctx);
        rgb_converter_ctx = nullptr;
    }

    pix_fmt = AVPixelFormat::AV_PIX_FMT_NONE;

}

void FFMPEG_Wrapper::RawCopyToRGB(AVFrame* dst, cimg_library::CImg<unsigned char>& src)
{
    int loc_ffmpeg = 0;

    for (int w = 0; w < dst->width; ++w)
    {
        for (int h = 0; h < dst->height; ++h)
        {
            loc_ffmpeg = src.spectrum() * (w + h * dst->width);

            dst->data[0][loc_ffmpeg] = src(w, h, 0, 0);
            dst->data[0][loc_ffmpeg + 1] = src(w, h, 0, 1);
            dst->data[0][loc_ffmpeg + 2] = src(w, h, 0, 2);
        }
    }
}

void FFMPEG_Wrapper::RawCopyToCImg(cimg_library::CImg<unsigned char>& dst, AVFrame* src)
{
    int loc_ffmpeg = 0;

    dst.assign(src->width, src->height, 1, 3);

    for (int w = 0; w < src->width; ++w)
    {
        for (int h = 0; h < src->height; ++h)
        {
            loc_ffmpeg = dst.spectrum() * (w + h * src->width);

            dst(w, h, 0, 0) = src->data[0][loc_ffmpeg];
            dst(w, h, 0, 1) = src->data[0][loc_ffmpeg + 1];
            dst(w, h, 0, 2) = src->data[0][loc_ffmpeg + 2];
        }
    }
}

bool FFMPEG_Wrapper::EncodeFrame(AVFrame* input_frame)
{
    result = avcodec_send_frame(codec_ctx, input_frame);
    if (result < 0)
    {
        std::cout << "Failed to encode frame on stream: " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    while (result >= 0)
    {
        result = avcodec_receive_packet(codec_ctx, packet);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
        {
            return true;
        }
        else if (result < 0) 
        {
            std::cout << "Failed to decode packet on stream: " << result << ", " << GetErrorString(result) << std::endl;
            return false;
        }

        std::cout << "Packing packet: " << current_packet << ", with PTS/DTS: " << packet->pts << "/" << packet->dts << std::endl;

        result = av_interleaved_write_frame(format_ctx, packet);
        if (result < 0)
        {
            std::cout << "Failed to write interleaved packet on stream: " << result << ", " << GetErrorString(result) << std::endl;
            return false;
        }
        else
        {
            ++current_packet;
        }
    }

    return true;
}

bool FFMPEG_Wrapper::DecodeFrame(AVFrame* output_frame)
{
    while (true)
    {
        result = av_read_frame(format_ctx, packet);
        if (result < 0)
        {
            return false;
        }

        if (packet->stream_index == reading_stream->index)
        {
            std::cout << "Packet PTS/DTS: " << packet->pts << "/" << packet->dts << std::endl;
            break;
        }
    }

    while (true)
    {
        result = avcodec_send_packet(codec_ctx, packet);
        if (result < 0)
        {
            std::cout << "Failed sending packet to codec: " << result << ", " << GetErrorString(result) << std::endl;
            return false;
        }

        result = avcodec_receive_frame(codec_ctx, other_format_frame);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
        {
            //std::cout << "OK error: " << result << ", " << GetErrorString(result) << std::endl;
            continue;
        }
        else if (result < 0)
        {
            std::cout << "Failed retreiving frame from codec: " << result << ", " << GetErrorString(result) << std::endl;
            return false;
        }

        //std::cout << "Read frame: " << current_frame << std::endl;
        ++current_frame;

        av_packet_unref(packet);
        return true;
    }
    

    return true;
}

bool FFMPEG_Wrapper::OpenReadingFile(std::string& filename)
{
    format_ctx = avformat_alloc_context();

    result = avformat_open_input(&format_ctx, filename.c_str(), nullptr, nullptr);
    if (result < 0) {
        std::cout << "Could not open input file: " << filename << ": " << result << ", " << GetErrorString(result) << std::endl;
        CloseReadingFile();
        return false;
    }

    return true;
}

bool FFMPEG_Wrapper::GetReadingStreams()
{
    result = avformat_find_stream_info(format_ctx, nullptr);
    if (result < 0) {
        std::cout << "Could not find stream info of input file: " << result << ", " << GetErrorString(result) << std::endl;
        CloseReadingFile();
        return false;
    }

    for (int i = 0; i < format_ctx->nb_streams; ++i)
    {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMediaType::AVMEDIA_TYPE_VIDEO)
        {
            reading_stream = format_ctx->streams[i];

            const AVCodec* codec = avcodec_find_decoder(reading_stream->codecpar->codec_id);
            if (!codec)
            {
                std::cout << "Invalid input stream for reading video" << std::endl;
                CloseReadingStreams();
                CloseReadingFile();
                return false;
            }

            codec_ctx = avcodec_alloc_context3(codec);

            std::cout << "Stream duration: " << reading_stream->duration << std::endl;
            std::cout << "Stream time base: " << reading_stream->time_base.num << ", " << reading_stream->time_base.den << std::endl;

            avcodec_parameters_to_context(codec_ctx, reading_stream->codecpar);
            avcodec_open2(codec_ctx, codec, NULL);

            break;
        }
    }

    std::cout << "Width: " << codec_ctx->width << std::endl;
    std::cout << "Height: " << codec_ctx->height << std::endl;
    std::cout << "Format: " << codec_ctx->pix_fmt << std::endl;
    std::cout << "Time Base: " << codec_ctx->time_base.num << ", " << codec_ctx->time_base.den << std::endl;

    if (!InitializeFrameInterop(codec_ctx->width, codec_ctx->height, codec_ctx->pix_fmt, SWS_FAST_BILINEAR, false))
    {
        CloseReadingStreams();
        CloseReadingFile();
    }

    return true;
}

bool FFMPEG_Wrapper::OpenWritingFile(std::string& filename, FFMPEG_WrapperStreamParams& params)
{
    result = avformat_alloc_output_context2(&format_ctx, nullptr, nullptr, filename.c_str());
    if (result < 0)
    {
        std::cout << "Could not open output file " << filename << ": " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    av_dict_set(&format_ctx->metadata, "preset", GetCompressionSpeedAsString(params.compression_speed).c_str(), 0);
    
    //format_ctx->duration_estimation_method = AVDurationEstimationMethod::AVFMT_DURATION_FROM_PTS;
    //format_ctx->duration = params.expected_stream_length;

    if (!InitializeFrameInterop(params.w, params.h, params.format, SWS_FAST_BILINEAR, true))
    {
        CloseWritingStream();
        CloseWritingFile();
    }


    return true;
}

bool FFMPEG_Wrapper::CreateWritingStream(FFMPEG_WrapperStreamParams& params)
{
    const AVCodec* encoder = avcodec_find_encoder(params.encoder_choice);
    if (!encoder)
    {
        std::cout << "Could not find encoder " << params.encoder_choice << std::endl;
        CloseWritingStream();
        CloseWritingFile();
        return false;
    }

    AVStream* stream = avformat_new_stream(format_ctx, encoder);
    if (!stream) {
        std::cout << "Error creating output stream" << std::endl;
        CloseWritingStream();
        CloseWritingFile();
        return false;
    }

    stream->time_base = av_inv_q(params.frames_per_second);

    codec_ctx = avcodec_alloc_context3(encoder);

    codec_ctx->width = params.w;
    codec_ctx->height = params.h;
    codec_ctx->pix_fmt = pix_fmt;
    codec_ctx->gop_size = params.frame_group_size;
    codec_ctx->max_b_frames = params.max_allowed_b_frames;
    codec_ctx->time_base = av_inv_q(params.frames_per_second);

    result = avcodec_open2(codec_ctx, encoder, nullptr);
    if (result < 0)
    {
        std::cout << "Could not open codec: " << result << ", " << GetErrorString(result) << std::endl;
        CloseWritingStream();
        CloseWritingFile();
        return false;
    }

    //av_dict_free(&codec_opt);

    result = avcodec_parameters_from_context(stream->codecpar, codec_ctx);
    if (result < 0)
    {
        std::cout << "Could not copy codec params: " << result << ", " << GetErrorString(result) << std::endl;
        CloseWritingStream();
        CloseWritingFile();
        return false;
    }

    //std::cout << "---STREAM metadata" << std::endl;
    //IterateOverDictionary(stream->metadata);

    return true;
}

bool FFMPEG_Wrapper::WriteHeader(std::string &filename)
{
    if (format_ctx->oformat->flags & AVFMT_GLOBALHEADER)
        codec_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

    //format_ctx->pb->

    //av_dict_set(&format_ctx->metadata, "title", "SomeTitle", 0);

    result = avio_open(&format_ctx->pb, filename.c_str(), AVIO_FLAG_WRITE);
    if (result < 0)
    {
        std::cout << "Could not open output file " << filename << ": " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    result = avformat_write_header(format_ctx, nullptr);
    if (result < 0)
    {
        std::cout << "Could not write header: " << result << ", " << GetErrorString(result) << std::endl;
        CloseWritingStream();
        CloseWritingFile();
        return false;
    }

    //std::cout << "---FORMAT_CTX metadata" << std::endl;
    //IterateOverDictionary(format_ctx->metadata);

    std::cout << "Stream time base: " << format_ctx->streams[0]->time_base.num << ", " << format_ctx->streams[0]->time_base.den << std::endl;
    std::cout << "Codec time base: " << codec_ctx->time_base.num << ", " << codec_ctx->time_base.den << std::endl;

    return true;
}

void FFMPEG_Wrapper::CloseReadingStreams()
{
    CleanupFrameInterop();

    if (codec_ctx) 
    {
        //avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
    }
}

void FFMPEG_Wrapper::CloseReadingFile()
{
    if (format_ctx)
    {
        avformat_close_input(&format_ctx);
    }
}

void FFMPEG_Wrapper::WriteTrailer()
{
    EncodeFrame(nullptr);

    //format_ctx->duration = 

    result = av_write_trailer(format_ctx);
    if (result < 0)
    {
        std::cout << "Failed to write trailer: " << result << ", " << GetErrorString(result) << std::endl;
    }

    result = avio_closep(&(format_ctx->pb));
    if (result < 0)
    {
        std::cout << "Failed to close file: " << result << ", " << GetErrorString(result) << std::endl;
    }


    IterateOverDictionary(format_ctx->metadata);
}

void FFMPEG_Wrapper::CloseWritingStream()
{
    if (codec_ctx)
    {
        //avcodec_close(codec_ctx);
        avcodec_free_context(&codec_ctx);
    }
}

void FFMPEG_Wrapper::CloseWritingFile()
{
    CleanupFrameInterop();

    avformat_free_context(format_ctx);
}

bool FFMPEG_Wrapper::CheckNotOpen()
{
    if (state != FFMPEG_WRAPPER_STATE::CLOSED)
    {
        std::cout << "File already opened on this wrapper" << std::endl;
        return false;
    }

    return true;
}

FFMPEG_Wrapper::FFMPEG_Wrapper()
{

}

FFMPEG_Wrapper::~FFMPEG_Wrapper()
{
    CloseVideoFile();
}

bool FFMPEG_Wrapper::OpenVideoFileForReading(std::string filename)
{
    if (!CheckNotOpen()) return false;

    if (!OpenReadingFile(filename)) return false;

    if (!GetReadingStreams()) return false;

    state = FFMPEG_WRAPPER_STATE::OPEN_READING;

    return true;
}

bool FFMPEG_Wrapper::OpenVideoFileForWriting(std::string filename, FFMPEG_WrapperStreamParams& params)
{
    if (!CheckNotOpen()) return false;

    if (!OpenWritingFile(filename, params)) return false;

    if (!CreateWritingStream(params)) return false;

    if (!WriteHeader(filename)) return false;

    state = FFMPEG_WRAPPER_STATE::OPEN_WRITING;

    return true;
}

bool FFMPEG_Wrapper::SaveFrame(cimg_library::CImg<unsigned char> &to_save)
{
    std::cout << "Saving frame " << current_frame << "..." << std::endl;

    result = av_frame_make_writable(rgb_frame);
    if (result < 0)
    {
        std::cout << "Failed to make RGB frame writable: " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    result = av_frame_make_writable(other_format_frame);
    if (result < 0)
    {
        std::cout << "Failed to make video frame writable: " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    RawCopyToRGB(rgb_frame, to_save);

    //result = sws_scale(rgb_converter_ctx, rgb_frame->data, rgb_frame->linesize, 0, rgb_frame->height, other_format_frame->data, other_format_frame->linesize);
    result = sws_scale_frame(rgb_converter_ctx, other_format_frame, rgb_frame);
    if (result < 0)
    {
        std::cout << "Failed to convert frame of stream: " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    std::cout << "Time Base of Frame:\n\t" 
        << other_format_frame->time_base.num << ", " << other_format_frame->time_base.den << std::endl;
    other_format_frame->pts = 
        current_frame * ((format_ctx->streams[0]->time_base.den * codec_ctx->time_base.num) / (codec_ctx->time_base.den * format_ctx->streams[0]->time_base.num));

    if (!EncodeFrame(other_format_frame))
    {
        return false;
    }

    ++current_frame;

    return true;
}

bool FFMPEG_Wrapper::RetrieveFrame(cimg_library::CImg<unsigned char>& to_load)
{
    if (!DecodeFrame(other_format_frame))
    {
        return false;
    }

    result = sws_scale_frame(rgb_converter_ctx, rgb_frame, other_format_frame);
    if (result < 0)
    {
        std::cout << "Failed to convert frame of stream: " << result << ", " << GetErrorString(result) << std::endl;
        return false;
    }

    RawCopyToCImg(to_load, rgb_frame);


    //std::cout << "Frame Stats:\n"
    //    << "Pict Type: " << other_format_frame->pict_type << "\n"
    //    << "PTS : " << other_format_frame->pts << "\n"
    //    << "Packet DTS: " << other_format_frame->pkt_dts << "\n"
    //    << "Key Frame: " << other_format_frame->key_frame << "\n"
    //    << std::endl;

    return true;
}

void FFMPEG_Wrapper::CloseVideoFile()
{
    switch (state)
    {
    case FFMPEG_WRAPPER_STATE::CLOSED:
        break;
    case FFMPEG_WRAPPER_STATE::OPEN_READING:
        CloseReadingStreams();
        CloseReadingFile();
        break;
    case FFMPEG_WRAPPER_STATE::OPEN_WRITING:
        WriteTrailer();
        CloseWritingStream();
        CloseWritingFile();
        break;
    }

    current_frame = 0;
    current_packet = 0;

    state = FFMPEG_WRAPPER_STATE::CLOSED;
}

//double FFMPEG_Wrapper::GetSSIM(cimg_library::CImg<unsigned char>& img1, cimg_library::CImg<unsigned char>& img2)
//{
//    return ;
//}
