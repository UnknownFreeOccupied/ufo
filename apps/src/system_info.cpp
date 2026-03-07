/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// UFO
#include <ufo/apps/system_info.hpp>
#include <ufo/compute/compute.hpp>

// STL
#include <bit>
#include <format>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <string_view>
#include <thread>

#if defined(_WIN32)
#include <windows.h>

#include <vector>
#pragma comment(lib, "advapi32.lib")
#elif defined(__APPLE__)
#include <mach/mach.h>
#include <sys/ioctl.h>
#include <sys/sysctl.h>
#include <sys/utsname.h>
#include <unistd.h>
#else
#include <sys/ioctl.h>
#include <sys/sysinfo.h>
#include <sys/utsname.h>
#include <unistd.h>
#endif

namespace
{

#if defined(_WIN32)
[[nodiscard]] OSVERSIONINFOEXW rtlOsVersion()
{
	using Fn = LONG(WINAPI*)(OSVERSIONINFOEXW*);
	auto fn  = reinterpret_cast<Fn>(
      GetProcAddress(GetModuleHandleA("ntdll.dll"), "RtlGetVersion"));
	OSVERSIONINFOEXW vi    = {};
	vi.dwOSVersionInfoSize = sizeof(vi);
	if (fn) fn(&vi);
	return vi;
}
#endif

#if defined(__linux__)
[[nodiscard]] std::string linuxName()
{
	std::ifstream file("/etc/os-release");
	std::string   line;
	if (file.is_open()) {
		while (std::getline(file, line)) {
			constexpr std::string_view key = "PRETTY_NAME=";
			if (line.starts_with(key)) {
				std::string name = line.substr(key.size());
				std::erase(name, '\"');
				return name;
			}
		}
	}
	return "Linux";
}
#endif

[[nodiscard]] std::string os()
{
#if defined(_WIN32)
	auto vi = rtlOsVersion();
	if (vi.dwMajorVersion != 0) {
		if (vi.dwMajorVersion == 10 && vi.dwBuildNumber >= 22000) return "Windows 11";
		if (vi.dwMajorVersion == 10) return "Windows 10";
		return std::format("Windows {}.{}", vi.dwMajorVersion, vi.dwMinorVersion);
	}
	return "Windows";
#elif defined(__APPLE__)
	char   ver[256] = {};
	size_t sz       = sizeof(ver);
	if (sysctlbyname("kern.osproductversion", ver, &sz, nullptr, 0) == 0) {
		std::string version(ver);
		int         major = std::stoi(version);

		auto name = [major]() -> std::string_view {
			switch (major) {
				case 11: return "Big Sur";
				case 12: return "Monterey";
				case 13: return "Ventura";
				case 14: return "Sonoma";
				case 15: return "Sequoia";
				default: return {};
			}
		}();
		if (!name.empty()) return std::format("macOS {} ({})", version, name);
		return "macOS " + version;
	}
	return "macOS";
#elif defined(__linux__)
	return linuxName();
#else
	return "Unknown";
#endif
}

[[nodiscard]] std::string cpuModel()
{
#if defined(_WIN32)
	HKEY hKey;
	if (RegOpenKeyExA(HKEY_LOCAL_MACHINE,
	                  "HARDWARE\\DESCRIPTION\\System\\CentralProcessor\\0", 0, KEY_READ,
	                  &hKey) == ERROR_SUCCESS) {
		char  buf[256] = {};
		DWORD sz       = sizeof(buf);
		if (RegQueryValueExA(hKey, "ProcessorNameString", nullptr, nullptr, (LPBYTE)buf,
		                     &sz) == ERROR_SUCCESS) {
			RegCloseKey(hKey);
			return std::string(buf);
		}
		RegCloseKey(hKey);
	}
	return "Unknown";
#elif defined(__APPLE__)
	char   buf[256] = {};
	size_t sz       = sizeof(buf);
	sysctlbyname("machdep.cpu.brand_string", buf, &sz, nullptr, 0);
	return std::string(buf);
#else
	std::ifstream file("/proc/cpuinfo");
	std::string   line;
	while (std::getline(file, line)) {
		if (line.starts_with("model name")) {
			auto pos = line.find(':');
			if (pos != std::string::npos) {
				auto name = line.substr(pos + 2);
				name.erase(name.find_last_not_of(" \t\r\n") + 1);
				return name;
			}
		}
	}
	return "Unknown";
#endif
}

[[nodiscard]] unsigned physicalCores()
{
#if defined(_WIN32)
	DWORD len = 0;
	GetLogicalProcessorInformation(nullptr, &len);
	std::vector<SYSTEM_LOGICAL_PROCESSOR_INFORMATION> buf(
	    len / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION));
	GetLogicalProcessorInformation(buf.data(), &len);
	unsigned count = 0;
	for (auto& info : buf) {
		if (info.Relationship == RelationProcessorCore) ++count;
	}
	return count;
#elif defined(__APPLE__)
	int    count = 0;
	size_t sz    = sizeof(count);
	sysctlbyname("hw.physicalcpu", &count, &sz, nullptr, 0);
	return static_cast<unsigned>(count);
#else
	std::ifstream                 file("/proc/cpuinfo");
	std::string                   line;
	std::set<std::pair<int, int>> cores;
	int                           physical_id = -1, core_id = -1;
	while (std::getline(file, line)) {
		if (line.starts_with("physical id")) {
			auto pos = line.find(':');
			if (pos != std::string::npos) physical_id = std::stoi(line.substr(pos + 1));
		} else if (line.starts_with("core id")) {
			auto pos = line.find(':');
			if (pos != std::string::npos) core_id = std::stoi(line.substr(pos + 1));
		} else if (line.empty() && physical_id >= 0 && core_id >= 0) {
			cores.insert({physical_id, core_id});
			physical_id = -1;
			core_id     = -1;
		}
	}
	// Handle last entry when file does not end with a blank line
	if (physical_id >= 0 && core_id >= 0) cores.insert({physical_id, core_id});

	return cores.empty() ? std::thread::hardware_concurrency()
	                     : static_cast<unsigned>(cores.size());
#endif
}

[[nodiscard]] auto cpuCores() { return std::thread::hardware_concurrency(); }

[[nodiscard]] std::string kernelVersion()
{
#if defined(_WIN32)
	auto vi = rtlOsVersion();
	if (vi.dwMajorVersion != 0) {
		return std::format("{}.{}.{}", vi.dwMajorVersion, vi.dwMinorVersion,
		                   vi.dwBuildNumber);
	}
	return "Unknown";
#else
	struct utsname buf;
	uname(&buf);
	return std::string(buf.release);
#endif
}

[[nodiscard]] std::string cpuArch()
{
#if defined(_WIN32)
	SYSTEM_INFO si;
	GetNativeSystemInfo(&si);
	switch (si.wProcessorArchitecture) {
		case PROCESSOR_ARCHITECTURE_AMD64: return "x86_64";
		case PROCESSOR_ARCHITECTURE_ARM: return "ARM";
		case PROCESSOR_ARCHITECTURE_ARM64: return "ARM64";
		case PROCESSOR_ARCHITECTURE_INTEL: return "x86";
		default: return "Unknown";
	}
#else
	struct utsname buf;
	uname(&buf);
	return std::string(buf.machine);
#endif
}

[[nodiscard]] auto physicalRam()
{
	std::size_t bytes = 0;
#if defined(_WIN32)
	MEMORYSTATUSEX status;
	status.dwLength = sizeof(status);
	GlobalMemoryStatusEx(&status);
	bytes = static_cast<std::size_t>(status.ullTotalPhys);
#elif defined(__APPLE__)
	int64_t mem = 0;
	size_t  len = sizeof(mem);
	sysctlbyname("hw.memsize", &mem, &len, nullptr, 0);
	bytes = static_cast<std::size_t>(mem);
#else
	struct sysinfo si;
	sysinfo(&si);
	bytes = static_cast<std::size_t>(si.totalram) * si.mem_unit;
#endif
	return static_cast<double>(bytes) / (1024 * 1024 * 1024);
}

[[nodiscard]] consteval std::size_t addressModel() { return sizeof(void*) * 8; }

[[nodiscard]] consteval std::string_view endianness()
{
	return (std::endian::native == std::endian::little) ? "Little Endian" : "Big Endian";
}

[[nodiscard]] std::string compiler()
{
#if defined(__clang__)
	return std::format("Clang {}.{}.{}", __clang_major__, __clang_minor__,
	                   __clang_patchlevel__);
#elif defined(__GNUC__)
	return std::format("GCC {}.{}.{}", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
#elif defined(_MSC_VER)
	return std::format("MSVC {}", _MSC_FULL_VER);
#else
	return "Unknown Compiler";
#endif
}

[[nodiscard]] double availableRam()
{
	std::size_t bytes = 0;
#if defined(_WIN32)
	MEMORYSTATUSEX status;
	status.dwLength = sizeof(status);
	GlobalMemoryStatusEx(&status);
	bytes = static_cast<std::size_t>(status.ullAvailPhys);
#elif defined(__APPLE__)
	vm_size_t              page_size = 0;
	mach_port_t            host      = mach_host_self();
	vm_statistics64_data_t vmstat    = {};
	mach_msg_type_number_t count     = HOST_VM_INFO64_COUNT;
	host_page_size(host, &page_size);
	if (host_statistics64(host, HOST_VM_INFO64, (host_info64_t)&vmstat, &count) ==
	    KERN_SUCCESS) {
		// free + inactive: inactive pages are reclaimable and count as available
		bytes = (static_cast<std::size_t>(vmstat.free_count) +
		         static_cast<std::size_t>(vmstat.inactive_count)) *
		        page_size;
	}
#else
	// MemAvailable accounts for free + reclaimable cache — same as `free -h`
	std::ifstream meminfo("/proc/meminfo");
	std::string   line;
	while (std::getline(meminfo, line)) {
		if (line.starts_with("MemAvailable:")) {
			auto pos = line.find(':');
			// Value is in kB
			bytes = std::stoull(line.substr(pos + 1)) * 1024;
			break;
		}
	}
#endif
	return static_cast<double>(bytes) / (1024 * 1024 * 1024);
}

[[nodiscard]] consteval std::string_view buildType()
{
#ifdef NDEBUG
	return "Release";
#else
	return "Debug";
#endif
}

[[nodiscard]] std::string simdExtensions()
{
	std::string ext;
#if defined(__AVX512F__)
	ext += "AVX-512F ";
#endif
#if defined(__AVX512BW__)
	ext += "AVX-512BW ";
#endif
#if defined(__AVX512DQ__)
	ext += "AVX-512DQ ";
#endif
#if defined(__AVX2__)
	ext += "AVX2 ";
#endif
#if defined(__FMA__)
	ext += "FMA ";
#endif
#if defined(__AVX__)
	ext += "AVX ";
#endif
#if defined(__SSE4_2__)
	ext += "SSE4.2 ";
#endif
#if defined(__SSE4_1__)
	ext += "SSE4.1 ";
#endif
#if defined(__SSSE3__)
	ext += "SSSE3 ";
#endif
#if defined(__SSE3__)
	ext += "SSE3 ";
#endif
#if defined(__SSE2__)
	ext += "SSE2 ";
#endif
#if defined(__ARM_FEATURE_SVE2)
	ext += "SVE2 ";
#endif
#if defined(__ARM_FEATURE_SVE)
	ext += "SVE ";
#endif
#if defined(__ARM_NEON)
	ext += "NEON ";
#endif
	if (ext.empty()) return "None";
	ext.pop_back();  // remove trailing space
	return ext;
}

[[nodiscard]] consteval std::string_view cppVersion()
{
#if __cplusplus > 202302L
	return "C++26 or later";
#elif __cplusplus == 202302L
	return "C++23";
#elif __cplusplus > 202002L
	return "C++23 (Experimental/Draft)";
#elif __cplusplus == 202002L
	return "C++20";
#elif __cplusplus > 201703L
	return "C++20 (Experimental/Draft)";
#elif __cplusplus == 201703L
	return "C++17";
#elif __cplusplus > 201402L
	return "C++17 (Experimental/Draft)";
#elif __cplusplus == 201402L
	return "C++14";
#elif __cplusplus > 201103L
	return "C++14 (Experimental/Draft)";
#elif __cplusplus == 201103L
	return "C++11";
#else
	return "Pre-C++11 or unknown";
#endif
}

size_t getTerminalWidth()
{
#ifdef _WIN32
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	if (GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &csbi)) {
		return static_cast<size_t>(csbi.srWindow.Right - csbi.srWindow.Left + 1);
	}
#else
	struct winsize w;
	if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0) {
		return static_cast<size_t>(w.ws_col);
	}
#endif
	return 80;  // Fallback
}

std::string wrapAndIndent(std::string const& text, size_t indentWidth, size_t maxWidth)
{
	if (text.empty()) return "";
	std::string result;
	size_t      availableWidth = (maxWidth > indentWidth) ? maxWidth - indentWidth : 20;
	std::string indent(indentWidth, ' ');
	size_t      start = 0;

	while (start < text.length()) {
		if (start > 0) result += "\n" + indent;

		if (text.length() - start <= availableWidth) {
			result += text.substr(start);
			break;
		}

		size_t end = start + availableWidth;
		// Search ONLY for spaces to keep version numbers (dots) together
		size_t break_pos = text.find_last_of(' ', end);

		if (break_pos != std::string::npos && break_pos > start) {
			result += text.substr(start, break_pos - start);
			start = break_pos + 1;  // Skip the space
		} else {
			// If a single word is longer than the line, force break it
			result += text.substr(start, availableWidth);
			start += availableWidth;
		}
	}
	return result;
}

}  // namespace

std::string ufo::systemInfo()
{
	size_t const MAX_WIDTH   = std::clamp<size_t>(getTerminalWidth(), 60, 100);
	size_t const LABEL_WIDTH = 19;

	// ANSI Palette
	std::string const RESET  = "\033[0m";
	std::string const BOLD   = "\033[1m";
	std::string const CYAN   = "\033[36m";
	std::string const BLUE   = "\033[34m";
	std::string const PURPLE = "\033[35m";
	std::string const YELLOW = "\033[33m";
	std::string const RED    = "\033[31m";
	std::string const GREEN  = "\033[32m";

	// 1. Dynamic Borders
	std::string title_text = " SYSTEM INFO ";
	size_t      side_len =
      (MAX_WIDTH > title_text.length()) ? (MAX_WIDTH - title_text.length()) / 2 : 0;
	std::string header =
	    std::format("{}{:=<{}}{}{:=<{}}{}", BOLD + BLUE, "", side_len, title_text, "",
	                MAX_WIDTH - side_len - title_text.length(), RESET);

	// 2. RAM Status Check (assuming physicalRam() returns GB)
	// You might need to parse availableRam() if it's a string, or use a numeric version
	// For this example, I'm assuming a numeric 'avail_gb' exists
	double      total_gb         = physicalRam();
	double      avail_gb         = availableRam();
	double      ratio            = avail_gb / total_gb;
	std::string ram_status_color = (ratio < 0.15) ? RED : (ratio < 0.35) ? YELLOW : GREEN;

	// 3. GPU Section
	auto const  gpus = compute::gpusInfo();
	std::string gpu_str;
	for (auto const& gpu : gpus) {
		std::string type =
		    (gpu.type == "Unknown" || gpu.type.empty()) ? "" : std::format("({}) ", gpu.type);
		// Note: Formatting width adjusted to 28 to account for 9 hidden ANSI chars in label
		std::string label = std::format("{}GPU [{}]:{}", PURPLE, gpu.backend, RESET);
		std::string content =
		    std::format("{} {}[Driver: {}]", gpu.name, type, gpu.description);
		gpu_str +=
		    std::format("{:<28}{}\n", label, wrapAndIndent(content, LABEL_WIDTH, MAX_WIDTH));
	}

	// 4. Build Status
	std::string bType(buildType());
	std::string bColor = (bType == "Debug") ? RED : GREEN;

	return std::format(
	    "{}\n"
	    "{:<28}{}\n"                                 // OS
	    "{:<28}{}\n"                                 // CPU
	    "{:<28}{}\n"                                 // SIMD
	    "{:<28}{:.1f} GB Total ({}{:.1f} GB{})\n\n"  // RAM with status color
	    "{}\n"
	    "{:<28}{}\n"              // Env
	    "{:<28}{}\n"              // Compiler
	    "{:<28}{}{} [{} {}]{}\n"  // Build
	    "{}{}{}",
	    header, YELLOW + "OS:" + RESET,
	    wrapAndIndent(std::format("{} (Kernel: {})", os(), kernelVersion()), LABEL_WIDTH,
	                  MAX_WIDTH),
	    YELLOW + "CPU:" + RESET,
	    cpuModel() + " (" + std::to_string(physicalCores()) + "C/" +
	        std::to_string(cpuCores()) + "T) @ " + cpuArch(),
	    YELLOW + "SIMD Extensions:" + RESET,
	    wrapAndIndent(simdExtensions(), LABEL_WIDTH, MAX_WIDTH), YELLOW + "RAM:" + RESET,
	    total_gb, ram_status_color, avail_gb, RESET, gpu_str, CYAN + "Environment:" + RESET,
	    std::format("{}-bit | {}", addressModel(), endianness()),
	    CYAN + "Compiler:" + RESET,
	    std::format("{} | {} ({})", compiler(), cppVersion(), __cplusplus),
	    CYAN + "Build:" + RESET, bColor, bType, __DATE__, __TIME__, RESET, BOLD + BLUE,
	    std::string(MAX_WIDTH, '='), RESET);
}

// std::string ufo::systemInfo()
// {
// 	auto const gpus = compute::gpusInfo();

// 	std::string gpu_str;
// 	for (std::size_t i = 0; i < gpus.size(); ++i) {
// 		auto const& gpu = gpus[i];

// 		std::string type_display =
// 		    (gpu.type == "Unknown") ? "" : std::format("({}) ", gpu.type);

// 		std::string label = std::format("GPU [{}]", gpu.backend);

// 		gpu_str += std::format("{:<19}{} {}[Driver: {}]\n", label + ":", gpu.name,
// 		                       type_display, gpu.description);
// 	}

// 	return std::format(
// 	    "==================== SYSTEM INFO ====================\n"
// 	    "OS:                {} (Kernel: {})\n"
// 	    "CPU:               {} ({}C/{}T) @ {}\n"
// 	    "SIMD Extensions:   {}\n"
// 	    "RAM:               {:.1f} GB Total ({} Available)\n"
// 	    "\n"
// 	    "{}"
// 	    "\n"
// 	    "Environment:       {}-bit | {}\n"
// 	    "Compiler:          {} | {} ({})\n"
// 	    "Build:             {} [{} {}]\n"
// 	    "=====================================================",
// 	    os(), kernelVersion(), cpuModel(), physicalCores(), cpuCores(), cpuArch(),
// 	    simdExtensions(), physicalRam(), availableRam(), gpu_str, addressModel(),
// 	    endianness(), compiler(), cppVersion(), __cplusplus, buildType(), __DATE__,
// 	    __TIME__);
// }

// std::string ufo::systemInfo()
// {
// 	auto const gpus = compute::gpusInfo();

// 	std::string gpu_str;
// 	for (std::size_t i = 0; i < gpus.size(); ++i) {
// 		auto const& gpu = gpus[i];
// 		if (i > 0) {
// 			gpu_str += std::format("\n{:<19}", std::format("GPU {}:", i));
// 		}

// 		gpu_str += std::format("{} [{}]", gpu.name, gpu.backend);
// 		gpu_str += std::format("\n{:<23}Vendor:  {} ({})", " ", gpu.vendor, gpu.type);

// 		if (!gpu.architecture.empty()) {
// 			gpu_str += std::format("\n{:<23}Arch:    {}", " ", gpu.architecture);
// 		}

// 		if (!gpu.description.empty()) {
// 			gpu_str += std::format("\n{:<23}Driver:  {}", " ", gpu.description);
// 		}
// 	}

// 	return std::format(
// 	    "==================== SYSTEM INFO ====================\n"
// 	    "OS:                {}\n"
// 	    "Kernel:            {}\n"
// 	    "CPU:               {}\n"
// 	    "CPU Architecture:  {}\n"
// 	    "SIMD Extensions:   {}\n"
// 	    "Physical Cores:    {}\n"
// 	    "Logical Cores:     {}\n"
// 	    "Physical RAM:      {:.1f} GB\n"
// 	    "Available RAM:     {}\n"
// 	    "GPU:               {}\n"
// 	    "Address Model:     {}-bit\n"
// 	    "Endianness:        {}\n"
// 	    "Compiler:          {}\n"
// 	    "C++ Standard:      {} ({})\n"
// 	    "Build Type:        {}\n"
// 	    "Build Time:        {} {}\n"
// 	    "=====================================================",
// 	    os(), kernelVersion(), cpuModel(), cpuArch(), simdExtensions(), physicalCores(),
// 	    cpuCores(), physicalRam(), availableRam(), gpu_str, addressModel(), endianness(),
// 	    compiler(), cppVersion(), __cplusplus, buildType(), __DATE__, __TIME__);
// }
