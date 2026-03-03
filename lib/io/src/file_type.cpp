// UFO
#include <ufo/io/file_type.hpp>
#include <ufo/utility/string.hpp>

// STL
#include <filesystem>
#include <string>
#include <unordered_map>

namespace ufo
{
namespace io
{
static std::unordered_map<std::string, FileType> const extension_map{
    {".ufo", FileType::UFO},        //
    {".xyz", FileType::XYZ},        //
    {".xyzi", FileType::XYZI},      //
    {".xyzn", FileType::XYZN},      //
    {".xyzrgb", FileType::XYZRGB},  //
    {".pts", FileType::PTS},        //
    {".ply", FileType::PLY},        //
    {".pcd", FileType::PCD},        //
    {".obj", FileType::OBJ},        //
    {".jpeg", FileType::JPEG},      //
    {".jpg", FileType::JPEG},       //
    {".png", FileType::PNG},        //
    {".qtp", FileType::QTP}         //
};
}  // namespace io

FileType fileType(std::filesystem::path const& file)
{
	std::string ext = tolower(file.extension().string());

	if (auto it = io::extension_map.find(ext); io::extension_map.end() != it) {
		return it->second;
	} else {
		return FileType::UNKNOWN;
	}
}
}  // namespace ufo