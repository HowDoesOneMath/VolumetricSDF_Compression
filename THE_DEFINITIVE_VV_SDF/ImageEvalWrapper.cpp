#include "ImageEvalWrapper.h"

void ImageEvalWrapper::CImgToFFMPEG(cimg_library::CImg<unsigned char>& src, AVFrame* dst)
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

bool ImageEvalWrapper::InitializeFrame(AVFrame** to_init, int width, int height)
{
    *to_init = av_frame_alloc();

    (*to_init)->width = width;
    (*to_init)->height = height;
    (*to_init)->format = AVPixelFormat::AV_PIX_FMT_RGB24;

    int result = av_frame_get_buffer(*to_init, 0);
    if (result < 0) {
        std::cout << "Could not create frame: " << result << std::endl;
        return false;
    }

    return true;
}

bool ImageEvalWrapper::Initialize(int width, int height)
{
    if (!InitializeFrame(&frame1, width, height))
    {
        CleanUp();
        return false;
    }

    if (!InitializeFrame(&frame2, width, height))
    {
        CleanUp();
        return false;
    }

    return true;
}

void ImageEvalWrapper::CleanUp()
{
    if (frame1)
    {
        av_frame_free(&frame1);
    }

    if (frame2)
    {
        av_frame_free(&frame2);
    }

}
