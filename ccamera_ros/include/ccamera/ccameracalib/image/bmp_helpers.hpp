#ifndef BMP_HPP
#define BMP_HPP

#include <vector>
#include <fstream>
#include <uchar.h>
#include <stdint.h>

struct bitmap_file_header
{
   unsigned short type;
   unsigned int   size;
   unsigned short reserved1;
   unsigned short reserved2;
   unsigned int   off_bits;

   unsigned int struct_size()
   {
      return sizeof(type     ) +
             sizeof(size     ) +
             sizeof(reserved1) +
             sizeof(reserved2) +
             sizeof(off_bits ) ;
   }
};

struct bitmap_information_header
{
   unsigned int   size;
   unsigned int   width;
   unsigned int   height;
   unsigned short planes;
   unsigned short bit_count;
   unsigned int   compression;
   unsigned int   size_image;
   unsigned int   x_pels_per_meter;
   unsigned int   y_pels_per_meter;
   unsigned int   clr_used;
   unsigned int   clr_important;

   unsigned int struct_size()
   {
      return sizeof(size            ) +
             sizeof(width           ) +
             sizeof(height          ) +
             sizeof(planes          ) +
             sizeof(bit_count       ) +
             sizeof(compression     ) +
             sizeof(size_image      ) +
             sizeof(x_pels_per_meter) +
             sizeof(y_pels_per_meter) +
             sizeof(clr_used        ) +
             sizeof(clr_important   ) ;
   }
};

inline std::size_t file_size(const std::string& file_name)
{
   std::ifstream file(file_name.c_str(),std::ios::in | std::ios::binary);
   if (!file) return 0;
   file.seekg (0, std::ios::end);
   return static_cast<std::size_t>(file.tellg());
}


inline bool big_endian()
{
   unsigned int v = 0x01;

   return (1 != reinterpret_cast<char*>(&v)[0]);
}

inline unsigned short flip(const unsigned short& v)
{
   return ((v >> 8) | (v << 8));
}

inline unsigned int flip(const unsigned int& v)
{
   return (
            ((v & 0xFF000000) >> 0x18) |
            ((v & 0x000000FF) << 0x18) |
            ((v & 0x00FF0000) >> 0x08) |
            ((v & 0x0000FF00) << 0x08)
          );
}

template <typename T>
inline void read_from_stream(std::ifstream& stream,T& t)
{
   stream.read(reinterpret_cast<char*>(&t),sizeof(T));
}

inline void read_bfh(std::ifstream& stream, bitmap_file_header& bfh)
{
   read_from_stream(stream,bfh.type     );
   read_from_stream(stream,bfh.size     );
   read_from_stream(stream,bfh.reserved1);
   read_from_stream(stream,bfh.reserved2);
   read_from_stream(stream,bfh.off_bits );

   if (big_endian())
   {
      bfh.type      = flip(bfh.type     );
      bfh.size      = flip(bfh.size     );
      bfh.reserved1 = flip(bfh.reserved1);
      bfh.reserved2 = flip(bfh.reserved2);
      bfh.off_bits  = flip(bfh.off_bits );
   }
}

inline void read_bih(std::ifstream& stream,bitmap_information_header& bih)
{
   read_from_stream(stream,bih.size            );
   read_from_stream(stream,bih.width           );
   read_from_stream(stream,bih.height          );
   read_from_stream(stream,bih.planes          );
   read_from_stream(stream,bih.bit_count       );
   read_from_stream(stream,bih.compression     );
   read_from_stream(stream,bih.size_image      );
   read_from_stream(stream,bih.x_pels_per_meter);
   read_from_stream(stream,bih.y_pels_per_meter);
   read_from_stream(stream,bih.clr_used        );
   read_from_stream(stream,bih.clr_important   );

   if (big_endian())
   {
      bih.size        = flip(bih.size                 );
      bih.width       = flip(bih.width                );
      bih.height      = flip(bih.height               );
      bih.planes      = flip(bih.planes               );
      bih.bit_count   = flip(bih.bit_count            );
      bih.compression = flip(bih.compression          );
      bih.size_image  = flip(bih.size_image           );
      bih.x_pels_per_meter = flip(bih.x_pels_per_meter);
      bih.y_pels_per_meter = flip(bih.y_pels_per_meter);
      bih.clr_used      = flip(bih.clr_used           );
      bih.clr_important = flip(bih.clr_important      );
   }
}

#endif // BMP_HPP
