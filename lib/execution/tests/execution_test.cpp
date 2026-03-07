// UFO
#include <ufo/execution/execution.hpp>

// Catch2
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("Execution Concepts")
{
	using namespace ufo::execution;

	STATIC_CHECK(Sequenced<sequenced_policy>);
	STATIC_CHECK(Parallel<parallel_policy>);
	STATIC_CHECK(ParallelUnsequenced<parallel_unsequenced_policy>);

	STATIC_CHECK(STLBackend<stl_parallel_policy>);

#if defined(UFO_PAR_TBB)
	STATIC_CHECK(TBBBackend<tbb_parallel_policy>);
#endif

#if defined(UFO_PAR_OMP)
	STATIC_CHECK(OMPBackend<omp_parallel_policy>);
#endif

#if defined(UFO_PAR_GCD)
	STATIC_CHECK(GCDBackend<gcd_parallel_policy>);
#endif
}

TEST_CASE("Execution") {}