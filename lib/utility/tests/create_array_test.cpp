// UFO
#include <ufo/utility/create_array.hpp>

// STL
#include <array>
#include <string>
#include <type_traits>
#include <vector>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[CreateArray] Basic integer array")
{
	constexpr auto arr = ufo::createArray<5>(42);
	static_assert(std::is_same_v<decltype(arr), std::array<int, 5> const>);
	for (auto v : arr) {
		REQUIRE(v == 42);
	}
}

TEST_CASE("[CreateArray] Basic double array")
{
	constexpr auto arr = ufo::createArray<3>(3.14);
	static_assert(std::is_same_v<decltype(arr), std::array<double, 3> const>);
	for (auto v : arr) {
		REQUIRE(v == Catch::Approx(3.14));
	}
}

TEST_CASE("[CreateArray] String array")
{
	auto arr = ufo::createArray<4>(std::string("hello"));
	for (auto& v : arr) {
		REQUIRE(v == "hello");
	}
}

TEST_CASE("[CreateArray] Array of empty string")
{
	auto arr = ufo::createArray<2>(std::string(""));
	for (auto& v : arr) {
		REQUIRE(v.empty());
	}
}

TEST_CASE("[CreateArray] Array of vectors")
{
	auto arr = ufo::createArray<3>(std::vector<int>{1, 2});
	for (auto& v : arr) {
		REQUIRE(v.size() == 2);
		REQUIRE(v[0] == 1);
		REQUIRE(v[1] == 2);
	}
}

TEST_CASE("[CreateArray] Zero size array")
{
	auto arr = ufo::createArray<0>(42);
	REQUIRE(arr.empty());
}

TEST_CASE("[CreateArray] Constexpr usage")
{
	constexpr auto arr = ufo::createArray<2>(7);
	static_assert(arr[0] == 7);
	static_assert(arr[1] == 7);
}

TEST_CASE("[CreateArray] Custom type")
{
	struct Foo {
		int  x;
		bool operator==(Foo const& other) const { return x == other.x; }
	};
	auto arr = ufo::createArray<3>(Foo{5});
	for (auto& v : arr) {
		REQUIRE(v == Foo{5});
	}
}

TEST_CASE("[CreateArray] Edge cases")
{
	// Negative values
	auto arr1 = ufo::createArray<2>(-1);
	REQUIRE(arr1[0] == -1);
	REQUIRE(arr1[1] == -1);

	// Floating point edge
	auto arr2 = ufo::createArray<2>(0.0);
	REQUIRE(arr2[0] == Catch::Approx(0.0));
	REQUIRE(arr2[1] == Catch::Approx(0.0));

	// Large array
	auto arr3 = ufo::createArray<100>(1);
	for (auto v : arr3) {
		REQUIRE(v == 1);
	}
}

TEST_CASE("[CreateArray] Array of arrays")
{
	auto arr = ufo::createArray<2>(std::array<int, 2>{1, 2});
	for (auto& v : arr) {
		REQUIRE(v[0] == 1);
		REQUIRE(v[1] == 2);
	}
}
