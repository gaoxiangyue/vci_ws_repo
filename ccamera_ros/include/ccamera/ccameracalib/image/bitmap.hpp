#ifndef BITMAP_HPP
#define BITMAP_HPP

#include <png.h>
#include <memory>
#include <vector>
#include <uchar.h>
#include <stdint.h>
#include "bmp_helpers.hpp"

namespace ce {


enum EBitmapResult {
    BITMAP_OK,
    BITMAP_NOT_PNG,
    BITMAP_NOT_BMP,
    BITMAP_ERROR
};

class Bitmap
{
public:

    Bitmap();
    Bitmap(int width, int height, int bitDepth, std::shared_ptr<char> data);
    Bitmap(int width, int height, int bitDepth, const char *data); //allocates new memory

    void cloneTo(Bitmap &bmp);

    int load(const std::string &);
    int loadPNG(const std::string &);
    int loadBMP(const std::string &);

    int saveAsPNG(const std::string &);

    int width();
    int height();
    int bitDepth();
    std::shared_ptr<char> data();

protected:
    int mWidth;
    int mHeight;
    int mBitDepth;
    std::shared_ptr<char> mData;

    int size() { return mWidth * mHeight * mBitDepth / 8; }

private:
    std::vector<png_bytep> rowPointers(png_structp png_ptr, png_infop info_ptr);
    std::vector<png_bytep> row_pointers;
};


}

#endif // BITMAP_HPP
