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

// STL
#include <cstdio>
#include <cstdlib>
#include <format>
#include <string_view>

// CLI11
#include <CLI/CLI.hpp>

int main(int argc, char* argv[])
{
	std::string const app_description =
	    "ufo is a command-line tool for UFO.\nWebsite: "
	    "https://github.com/UnknownFreeOccupied/ufo";
	std::string const app_version =
	    std::format("UFO v{:d}.{:d}.{:d}\n\n{}", 1, 0, 0, ufo::systemInfo());
	constexpr std::string_view const app_footer_format =
	    "Call `ufo {} <COMMAND> -h` for more detailed usage";

	CLI::App app(app_description);

	app.set_version_flag("-v,--version", app_version)->group("INFO");

	app.footer(std::format(app_footer_format, " "));

	app.set_help_all_flag("--help-all", "Expand all help");

	app.get_formatter()->label("SUBCOMMAND", "COMMAND");
	app.get_formatter()->label("SUBCOMMANDS", "COMMANDS");

	app.require_subcommand(0);

	CLI11_PARSE(app, argc, argv);

	return EXIT_SUCCESS;
}