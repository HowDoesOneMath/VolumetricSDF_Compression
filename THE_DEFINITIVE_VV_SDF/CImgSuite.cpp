#include "CImgSuite.h"

void CImgSuite::SampleCImg()
{
    std::string myImage = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/BlueSmiley.png";
    std::string imageSaveName = "D:/VsprojectsOnD/_VV_PROJ/THE_DEFINITIVE_VV_SDF/THE_DEFINITIVE_VV_SDF/_ImageDump/_RandomTestImages/NEW_BlueSmiley.png";

    //cimg_library::CImg<unsigned char> image("lena.jpg"), visu(500, 400, 1, 3, 0);
    cimg_library::CImg<unsigned char> image(myImage.c_str()), visu(500, 400, 1, 3, 0);
    const unsigned char red[] = { 255,0,0 }, green[] = { 0,255,0 }, blue[] = { 0,0,255 };
    image.blur(2.5);
    cimg_library::CImgDisplay main_disp(image, "Click a point"), draw_disp(visu, "Intensity profile");
    while (!main_disp.is_closed() && !draw_disp.is_closed()) {
        main_disp.wait();
        if (main_disp.button() && main_disp.mouse_y() >= 0) {
            const int y = main_disp.mouse_y();
            visu.fill(0).draw_graph(image.get_crop(0, y, 0, 0, image.width() - 1, y, 0, 0), red, 1, 1, 0, 255, 0);
            visu.draw_graph(image.get_crop(0, y, 0, 1, image.width() - 1, y, 0, 1), green, 1, 1, 0, 255, 0);
            visu.draw_graph(image.get_crop(0, y, 0, 2, image.width() - 1, y, 0, 2), blue, 1, 1, 0, 255, 0).display(draw_disp);
        }
    }
    //return 0;

    image.save_png(imageSaveName.c_str());
}

void CImgSuite::run(int argc, char** argv)
{
    SampleCImg();
}
