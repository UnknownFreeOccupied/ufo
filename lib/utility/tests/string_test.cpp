// UFO
#include <ufo/utility/string.hpp>

// STL
#include <string>
#include <vector>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[String] split")
{
	std::string s      = "a,b,c";
	auto        result = ufo::split(s, ',');
	REQUIRE(result.size() == 3);
	REQUIRE(result[0] == "a");
	REQUIRE(result[1] == "b");
	REQUIRE(result[2] == "c");
}

TEST_CASE("[String] join")
{
	std::vector<std::string> v      = {"a", "b", "c"};
	auto                     result = ufo::join(v, ',');
	REQUIRE(result == "a,b,c");
}

TEST_CASE("[String] startsWith")
{
	REQUIRE(ufo::startsWith("abcdef", "abc"));
	REQUIRE_FALSE(ufo::startsWith("abcdef", "def"));
}

TEST_CASE("[String] endsWith")
{
	REQUIRE(ufo::endsWith("abcdef", "def"));
	REQUIRE_FALSE(ufo::endsWith("abcdef", "abc"));
}

TEST_CASE("[String] contains")
{
	REQUIRE(ufo::contains("abcdef", "cd"));
	REQUIRE_FALSE(ufo::contains("abcdef", "gh"));
}

TEST_CASE("[String] tolower/toupper")
{
	REQUIRE(ufo::tolower("ABCdef") == "abcdef");
	REQUIRE(ufo::toupper("abcDEF") == "ABCDEF");
}

TEST_CASE("[String] ltrim/rtrim/trim")
{
	std::string s1 = "   abc";
	ufo::ltrim(s1);
	REQUIRE(s1 == "abc");
	std::string s2 = "abc   ";
	ufo::rtrim(s2);
	REQUIRE(s2 == "abc");
	std::string s3 = "   abc   ";
	ufo::trim(s3);
	REQUIRE(s3 == "abc");
}

TEST_CASE("[String] ltrimCopy/rtrimCopy/trimCopy")
{
	REQUIRE(ufo::ltrimCopy("   abc") == "abc");
	REQUIRE(ufo::rtrimCopy("abc   ") == "abc");
	REQUIRE(ufo::trimCopy("   abc   ") == "abc");
}

TEST_CASE("[String] split edge cases")
{
	REQUIRE(ufo::split("", ',').empty());
	REQUIRE(ufo::split(",", ',').size() == 1);  // TODO: Should this return 1 or 2?
	REQUIRE(ufo::split("a,,b", ',').size() == 3);
	REQUIRE(ufo::split("a b c", ' ').size() == 3);
	REQUIRE(ufo::split("abc", ',').size() == 1);
}

TEST_CASE("[String] join edge cases")
{
	REQUIRE(ufo::join({}, ',') == "");
	REQUIRE(ufo::join({""}, ',') == "");
	REQUIRE(ufo::join({"a", "", "b"}, ',') == "a,,b");
}

TEST_CASE("[String] startsWith/endsWith/contains edge cases")
{
	REQUIRE(ufo::startsWith("", ""));
	REQUIRE(ufo::startsWith("abc", ""));
	REQUIRE_FALSE(ufo::startsWith("", "abc"));
	REQUIRE(ufo::endsWith("", ""));
	REQUIRE(ufo::endsWith("abc", ""));
	REQUIRE_FALSE(ufo::endsWith("", "abc"));
	REQUIRE(ufo::contains("", ""));
	REQUIRE_FALSE(ufo::contains("", "abc"));
	REQUIRE(ufo::contains("abc", ""));
}

TEST_CASE("[String] tolower/toupper edge cases")
{
	REQUIRE(ufo::tolower("") == "");
	REQUIRE(ufo::toupper("") == "");
	REQUIRE(ufo::tolower("123!@#") == "123!@#");
	REQUIRE(ufo::toupper("123!@#") == "123!@#");
}

TEST_CASE("[String] ltrim/rtrim/trim whitespace cases")
{
	std::string s1 = "\t\n abc";
	ufo::ltrim(s1);
	REQUIRE(s1 == "abc");
	std::string s2 = "abc \t\n";
	ufo::rtrim(s2);
	REQUIRE(s2 == "abc");
	std::string s3 = "\t\n abc \t\n";
	ufo::trim(s3);
	REQUIRE(s3 == "abc");
	std::string s4 = "abc";
	ufo::trim(s4);
	REQUIRE(s4 == "abc");
}

TEST_CASE("[String] ltrimCopy/rtrimCopy/trimCopy whitespace cases")
{
	REQUIRE(ufo::ltrimCopy("\t\n abc") == "abc");
	REQUIRE(ufo::rtrimCopy("abc \t\n") == "abc");
	REQUIRE(ufo::trimCopy("\t\n abc \t\n") == "abc");
	REQUIRE(ufo::trimCopy("abc") == "abc");
}