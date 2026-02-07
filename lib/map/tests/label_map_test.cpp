// UFO
#include <ufo/map/label/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("LabelMap")
{
	SECTION("Label set")
	{
		Map3D<LabelMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);

		map.labelSet(coord, 5);

		auto l = map.label(coord);
		REQUIRE(l == 5);

		l = map.label(coord_r);
		REQUIRE(l == l);
	}

	SECTION("Label Propagation")
	{
		Map3D<LabelMap> map(0.5, 3);

		Vec3f     coord1(0, 0, 0);
		Vec3f     coord2(0, 0, 0.6);
		TreeCoord coord_r(coord1, 2);

		map.labelSet(coord1, 5);

		auto l = map.label(coord1);
		REQUIRE(l == 5);

		l = map.label(coord_r);
		REQUIRE(l == 5);

		map.labelSet(coord2, 4);

		l = map.label(coord2);
		REQUIRE(l == 4);

		l = map.label(coord_r);
		REQUIRE(l == (4 | 5));
	}
}
