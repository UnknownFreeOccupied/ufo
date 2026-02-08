// UFO
#include <ufo/map/occupancy/map.hpp>
#include <ufo/map/ufomap.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("OccupancyMap")
{
	SECTION("Occupancy Set")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f coord(0, 0, 0);
		float occ   = 0.95f;
		float logit = probabilityToLogit(occ);

		map.occupancySet(coord, occ);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(occ == Catch::Approx(map.occupancy(coord)));
		REQUIRE(logit == Catch::Approx(map.occupancyLogit(coord)));
	}

	SECTION("Occupancy Logit Set")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f coord(0, 0, 0);
		float occ   = 0.95f;
		float logit = probabilityToLogit(occ);

		map.occupancySetLogit(coord, logit);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(occ == Catch::Approx(map.occupancy(coord)));
		REQUIRE(logit == Catch::Approx(map.occupancyLogit(coord)));
	}

	SECTION("Occupancy Update")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord)));
		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.occupancyUpdate(coord, occ, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(occ == Catch::Approx(map.occupancy(coord)));

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.propagate();

		REQUIRE(occ == Catch::Approx(map.occupancy(coord_r)));

		float occ2   = 0.8f;
		float logit2 = probabilityToLogit(occ2);

		map.occupancyUpdateLogit(coord, logit2, false);

		REQUIRE(logitToProbability(logit + logit2) == Catch::Approx(map.occupancy(coord)));
		REQUIRE(occ == Catch::Approx(map.occupancy(coord_r)));

		map.propagate();

		REQUIRE(logitToProbability(logit + logit2) == Catch::Approx(map.occupancy(coord_r)));

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
	}

	SECTION("Occupancy Update With Clamping")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord)));
		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.occupancyUpdate(coord, occ, 0.7f, 0.9f, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(0.9f == Catch::Approx(map.occupancy(coord)));

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.propagate();

		REQUIRE(0.9f == Catch::Approx(map.occupancy(coord_r)));
	}

	SECTION("Occupancy Update UnaryOp")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord)));

		map.occupancyUpdate(coord, [occ](TreeIndex) { return occ; }, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(occ == Catch::Approx(map.occupancy(coord)));
		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.propagate();

		REQUIRE(occ == Catch::Approx(map.occupancy(coord_r)));
	}

	SECTION("Occupancy Update Logit UnaryOp")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord)));

		map.occupancyUpdateLogit(coord, [logit](TreeIndex) { return logit; }, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(occ == Catch::Approx(map.occupancy(coord)));
		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.propagate();

		REQUIRE(occ == Catch::Approx(map.occupancy(coord_r)));
	}

	SECTION("Occupancy Update Logit with Clamping")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord)));

		map.occupancyUpdateLogit(coord, logit, probabilityToLogit(0.7f),
		                         probabilityToLogit(0.9f), false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(0.9f == Catch::Approx(map.occupancy(coord)));
		REQUIRE(0.5f == Catch::Approx(map.occupancy(coord_r)));

		map.propagate();

		REQUIRE(0.9f == Catch::Approx(map.occupancy(coord_r)));
	}

	SECTION("Occupancy Threshold Set")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		float occ_thres  = 0.7f;
		float free_thres = 0.3f;

		map.occupancySetThres(occ_thres, free_thres);

		REQUIRE(occ_thres == Catch::Approx(map.occupiedThres()));
		REQUIRE(free_thres == Catch::Approx(map.freeThres()));
	}

	SECTION("Occupancy Threshold Logit Set")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		float occ_thres  = 0.7f;
		float free_thres = 0.3f;

		map.occupancySetThresLogit(occ_thres, free_thres);

		REQUIRE(occ_thres == Catch::Approx(map.occupiedThresLogit()));
		REQUIRE(free_thres == Catch::Approx(map.freeThresLogit()));
	}
}