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
		REQUIRE(map.occupancy(coord) == Catch::Approx(occ));
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
		REQUIRE(map.occupancy(coord) == Catch::Approx(occ));
		REQUIRE(map.occupancyLogit(coord) == Catch::Approx(logit));
	}

	SECTION("Occupancy Update")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2u);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(map.occupancy(coord) == Catch::Approx(0.5f));
		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.occupancyUpdate(coord, occ, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(map.occupancy(coord) == Catch::Approx(occ));

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.propagate();

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(occ));

		float occ2   = 0.8f;
		float logit2 = probabilityToLogit(occ2);

		map.occupancyUpdateLogit(coord, logit2, false);

		REQUIRE(map.occupancy(coord) == Catch::Approx(logitToProbability(logit + logit2)));
		REQUIRE(map.occupancy(coord_r) == Catch::Approx(occ));

		map.propagate();

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(logitToProbability(logit + logit2)));

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
	}

	SECTION("Occupancy Update With Clamping")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2u);
		float     occ = 0.95f;

		REQUIRE(map.occupancy(coord) == Catch::Approx(0.5f));
		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.occupancyUpdate(coord, occ, 0.7f, 0.9f, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(map.occupancy(coord) == Catch::Approx(0.9f));

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.propagate();

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.9f));
	}

	SECTION("Occupancy Update UnaryOp")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2u);
		float     occ = 0.95f;

		REQUIRE(map.occupancy(coord) == Catch::Approx(0.5f));

		map.occupancyUpdate(coord, [occ](TreeIndex) { return occ; }, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(map.occupancy(coord) == Catch::Approx(occ));
		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.propagate();

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(occ));
	}

	SECTION("Occupancy Update Logit UnaryOp")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2u);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(map.occupancy(coord) == Catch::Approx(0.5f));

		map.occupancyUpdateLogit(coord, [logit](TreeIndex) { return logit; }, false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(map.occupancy(coord) == Catch::Approx(occ));
		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.propagate();

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(occ));
	}

	SECTION("Occupancy Update Logit with Clamping")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		Vec3f     coord(0, 0, 0);
		TreeCoord coord_r(coord, 2u);
		float     occ   = 0.95f;
		float     logit = probabilityToLogit(occ);

		REQUIRE(map.occupancy(coord) == Catch::Approx(0.5f));

		map.occupancyUpdateLogit(coord, logit, probabilityToLogit(0.7f),
		                         probabilityToLogit(0.9f), false);

		REQUIRE(!map.containsUnknown(coord));
		REQUIRE(!map.containsFree(coord));
		REQUIRE(map.containsOccupied(coord));
		REQUIRE(map.occupancy(coord) == Catch::Approx(0.9f));
		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.5f));

		map.propagate();

		REQUIRE(map.occupancy(coord_r) == Catch::Approx(0.9f));
	}

	SECTION("Occupancy Threshold Set")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		float occ_thres  = 0.7f;
		float free_thres = 0.3f;

		float occ_thres_logit  = probabilityToLogit(occ_thres);
		float free_thres_logit = probabilityToLogit(free_thres);

		map.occupancySetThres(occ_thres, free_thres);

		REQUIRE(map.occupiedThres() == Catch::Approx(occ_thres));
		REQUIRE(map.freeThres() == Catch::Approx(free_thres));

		REQUIRE(map.occupiedThresLogit() == Catch::Approx(occ_thres_logit));
		REQUIRE(map.freeThresLogit() == Catch::Approx(free_thres_logit));
	}

	SECTION("Occupancy Threshold Logit Set")
	{
		Map3D<OccupancyMap> map(0.5, 3);

		float occ_thres  = 0.7f;
		float free_thres = 0.3f;

		float occ_thres_logit  = probabilityToLogit(occ_thres);
		float free_thres_logit = probabilityToLogit(free_thres);

		map.occupancySetThresLogit(occ_thres_logit, free_thres_logit);

		REQUIRE(map.occupiedThresLogit() == Catch::Approx(occ_thres_logit));
		REQUIRE(map.freeThresLogit() == Catch::Approx(free_thres_logit));

		REQUIRE(map.occupiedThres() == Catch::Approx(occ_thres));
		REQUIRE(map.freeThres() == Catch::Approx(free_thres));
	}
}