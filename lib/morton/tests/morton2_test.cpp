// UFO
#include <ufo/morton/morton2.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <iostream>

TEST_CASE("Morton2")
{
	ufo::Vec2u v(0, 1);

	auto c = ufo::Morton<v.size()>::encode(v);

	auto v2 = ufo::Morton<v.size()>::decode(c);

	std::cout << all(v == v2) << std::endl;
}