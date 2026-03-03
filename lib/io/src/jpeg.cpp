// UFO
#include <ufo/io/jpeg.hpp>

// STL
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <format>
#include <stdexcept>

// JPEG
#include <jerror.h>
#include <jpeglib.h>  // Include after cstddef to define size_t

namespace ufo
{
namespace detail
{
void jpeg_error_throw(j_common_ptr p_cinfo)
{
	if (p_cinfo->is_decompressor) {
		jpeg_destroy_decompress(reinterpret_cast<jpeg_decompress_struct*>(p_cinfo));
	} else {
		jpeg_destroy_compress(reinterpret_cast<jpeg_compress_struct*>(p_cinfo));
	}

	char buffer[JMSG_LENGTH_MAX];
	(*p_cinfo->err->format_message)(p_cinfo, buffer);
	throw std::runtime_error(buffer);
}

bool readJPEG(FileHandler fp, std::uint8_t* image)
{
	jpeg_decompress_struct cinfo;
	jpeg_error_mgr         jerr;
	JSAMPARRAY             buffer;

	try {
		cinfo.err       = jpeg_std_error(&jerr);
		jerr.error_exit = jpeg_error_throw;
		jpeg_create_decompress(&cinfo);
		jpeg_stdio_src(&cinfo, fp.get());
		jpeg_read_header(&cinfo, TRUE);

		// We only support two channel types: RGB and Gray
		int num_of_channels   = 3;
		int bytes_per_channel = 1;
		switch (cinfo.jpeg_color_space) {
			case JCS_RGB:
			case JCS_YCbCr:
				cinfo.out_color_space      = JCS_RGB;
				cinfo.out_color_components = 3;
				num_of_channels            = 3;
				break;
			case JCS_GRAYSCALE:
				cinfo.jpeg_color_space     = JCS_GRAYSCALE;
				cinfo.out_color_components = 1;
				num_of_channels            = 1;
				break;
			case JCS_CMYK:
			case JCS_YCCK:
			default:
				std::println(stderr, "[UFO | Read JPEG] Color space not supported");
				jpeg_destroy_decompress(&cinfo);
				fp.close();
				return false;
		}

		jpeg_start_decompress(&cinfo);
		int row_stride = cinfo.output_width * cinfo.output_components;
		buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr)&cinfo, JPOOL_IMAGE, row_stride, 1);
		while (cinfo.output_scanline < cinfo.output_height) {
			jpeg_read_scanlines(&cinfo, buffer, 1);
			std::memcpy(image, buffer[0], row_stride);
			image += row_stride;
		}
		jpeg_finish_decompress(&cinfo);
		jpeg_destroy_decompress(&cinfo);
		fp.close();
		return true;
	} catch (std::runtime_error const& err) {
		std::println(stderr, "[UFO | Read JPEG] libjpeg error: {}", err.what());
		fp.close();
		return false;
	}
}

bool writeJPEG(FileHandler& fp, std::uint8_t const* image, std::uint32_t width,
               std::uint32_t height, int num_channels, int quality)
{
	jpeg_compress_struct cinfo;
	jpeg_error_mgr       jerr;
	JSAMPROW             row_pointer[1];

	try {
		cinfo.err       = jpeg_std_error(&jerr);
		jerr.error_exit = jpeg_error_throw;

		jpeg_create_compress(&cinfo);
		jpeg_stdio_dest(&cinfo, fp.get());

		cinfo.image_width      = width;
		cinfo.image_height     = height;
		cinfo.input_components = num_channels;
		cinfo.in_color_space   = 1 == num_channels ? JCS_GRAYSCALE : JCS_RGB;

		jpeg_set_defaults(&cinfo);
		jpeg_set_quality(&cinfo, quality, TRUE);
		jpeg_start_compress(&cinfo, TRUE);

		int                       row_stride = width * num_channels;
		std::vector<std::uint8_t> buffer(row_stride);
		while (cinfo.next_scanline < cinfo.image_height) {
			std::memcpy(buffer.data(), image, row_stride);
			row_pointer[0] = buffer.data();
			jpeg_write_scanlines(&cinfo, row_pointer, 1);
			image += row_stride;
		}

		jpeg_finish_compress(&cinfo);
		fp.close();
		jpeg_destroy_compress(&cinfo);

		return true;
	} catch (std::runtime_error const& err) {
		std::println(stderr, "[UFO | Write JPEG] Error: {}", err.what());
		fp.close();

		return false;
	}
}
}  // namespace detail

ImageProperties imagePropertiesJPEG(std::filesystem::path const& file)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Image Properties JPEG] Failed to open file: {}", file.string()));
	}

	jpeg_decompress_struct cinfo;
	jpeg_error_mgr         jerr;

	try {
		cinfo.err       = jpeg_std_error(&jerr);
		jerr.error_exit = detail::jpeg_error_throw;
		jpeg_create_decompress(&cinfo);
		jpeg_stdio_src(&cinfo, fp.get());
		jpeg_read_header(&cinfo, TRUE);
	} catch (std::runtime_error const& err) {
		throw std::runtime_error(
		    std::format("[UFO | Read JPEG] libjpeg error: {}", err.what()));
		fp.close();
	}

	if (JCS_GRAYSCALE != cinfo.jpeg_color_space && JCS_RGB != cinfo.jpeg_color_space &&
	    JCS_YCbCr != cinfo.jpeg_color_space) {
		jpeg_destroy_decompress(&cinfo);
		fp.close();
		throw std::runtime_error("[UFO | Read JPEG] Color space not supported");
	}

	ImageProperties prop;
	prop.width     = cinfo.image_width;
	prop.height    = cinfo.image_height;
	prop.bit_depth = 8;
	prop.alpha     = false;
	prop.grayscale = cinfo.jpeg_color_space == JCS_GRAYSCALE;

	jpeg_destroy_decompress(&cinfo);

	return prop;
}
}  // namespace ufo