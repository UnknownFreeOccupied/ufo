// UFO
#include <ufo/execution/algorithm.hpp>
#include <ufo/execution/execution.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <numeric>
#include <utility>
#include <vector>

using namespace ufo;

struct Data {
	int value;
};

template <class Policy>
void test_for_each(Policy&& policy)
{
	std::size_t const size = 1000;
	std::vector<int>  v(size);
	std::iota(v.begin(), v.end(), 0);

	std::vector<int> results(size, 0);

	ufo::for_each(std::forward<Policy>(policy), std::size_t(0), size,
	              [&results](std::size_t i) { results[i] = 1; });

	for (auto r : results) {
		CHECK(r == 1);
	}

	std::fill(results.begin(), results.end(), 0);
	ufo::for_each(std::forward<Policy>(policy), v, [&results](int i) { results[i] = 2; });

	for (auto r : results) {
		CHECK(r == 2);
	}

	// Range result check
	auto f_fe = [](int) {};
	auto res  = ufo::for_each(v, f_fe);
	static_assert(
	    std::is_same_v<decltype(res), std::ranges::for_each_result<
	                                      std::vector<int>::iterator, decltype(f_fe)>>);
}

template <class Policy>
void test_transform_unary(Policy&& policy)
{
	std::size_t const size = 1000;
	std::vector<int>  v(size);
	std::iota(v.begin(), v.end(), 0);

	std::vector<int> results(size);

	auto it = ufo::transform(std::forward<Policy>(policy), v.begin(), v.end(),
	                         results.begin(), [](int i) { return i * 2; });
	CHECK(it == results.end());

	for (std::size_t i = 0; i < size; ++i) {
		CHECK(results[i] == static_cast<int>(i * 2));
	}

	std::fill(results.begin(), results.end(), 0);
	it = ufo::transform(std::forward<Policy>(policy), v, results.begin(),
	                    [](int i) { return i * 3; });
	CHECK(it == results.end());

	for (std::size_t i = 0; i < size; ++i) {
		CHECK(results[i] == static_cast<int>(i * 3));
	}

	// Range result check
	auto f   = [](int i) { return i; };
	auto res = ufo::transform(v, results.begin(), f);
	static_assert(
	    std::is_same_v<decltype(res),
	                   std::ranges::unary_transform_result<std::vector<int>::iterator,
	                                                       std::vector<int>::iterator>>);
}

template <class Policy>
void test_transform_binary(Policy&& policy)
{
	std::size_t const size = 1000;
	std::vector<int>  v1(size);
	std::vector<int>  v2(size);
	std::iota(v1.begin(), v1.end(), static_cast<int>(0));
	std::iota(v2.begin(), v2.end(), static_cast<int>(size));

	std::vector<int> results(size);

	auto it = ufo::transform(std::forward<Policy>(policy), v1.begin(), v1.end(), v2.begin(),
	                         results.begin(), [](int a, int b) { return a + b; });
	CHECK(it == results.end());

	for (std::size_t i = 0; i < size; ++i) {
		CHECK(results[i] == static_cast<int>(i + (size + i)));
	}

	std::fill(results.begin(), results.end(), 0);
	it = ufo::transform(std::forward<Policy>(policy), v1, v2.begin(), results.begin(),
	                    [](int a, int b) { return a - b; });
	CHECK(it == results.end());

	for (std::size_t i = 0; i < size; ++i) {
		CHECK(results[i] == static_cast<int>(i - (size + i)));
	}

	// Range result check
	auto f_tr2 = [](int a, int b) { return a + b; };
	auto res   = ufo::transform(v1, v2.begin(), results.begin(), f_tr2);
	static_assert(
	    std::is_same_v<decltype(res),
	                   std::ranges::binary_transform_result<std::vector<int>::iterator,
	                                                        std::vector<int>::iterator,
	                                                        std::vector<int>::iterator>>);
}

TEST_CASE("Algorithm - Projections and Invoke")
{
	std::size_t const size = 10;
	std::vector<Data> v(size);
	for (std::size_t i = 0; i < size; ++i) v[i].value = i;

	SECTION("Sequential - for_each")
	{
		int sum = 0;
		ufo::for_each(v, [&](int val) { sum += val; }, &Data::value);
		CHECK(sum == 45);
	}

	SECTION("Sequential - transform")
	{
		std::vector<int> results(size);
		auto             res =
		    ufo::transform(v, results.begin(), [](int val) { return val * 2; }, &Data::value);
		CHECK(res.out == results.end());
		for (std::size_t i = 0; i < size; ++i) CHECK(results[i] == static_cast<int>(i * 2));
	}

	SECTION("Parallel - for_each")
	{
		std::atomic<int> sum = 0;
		ufo::for_each(execution::par, v, [&](int val) { sum += val; }, &Data::value);
		CHECK(sum == 45);
	}

	SECTION("Parallel - transform")
	{
		std::vector<int> results(size);
		auto             res = ufo::transform(
        execution::par, v, results.begin(), [](int val) { return val * 2; },
        &Data::value);
		CHECK(res == results.end());
		for (std::size_t i = 0; i < size; ++i) CHECK(results[i] == static_cast<int>(i * 2));
	}
}

TEST_CASE("Algorithm - Sequential")
{
	SECTION("for_each") { test_for_each(execution::seq); }
	SECTION("transform unary") { test_transform_unary(execution::seq); }
	SECTION("transform binary") { test_transform_binary(execution::seq); }
}

TEST_CASE("Algorithm - Parallel STL")
{
	SECTION("for_each") { test_for_each(execution::par); }
	SECTION("transform unary") { test_transform_unary(execution::par); }
	SECTION("transform binary") { test_transform_binary(execution::par); }
}

#if defined(UFO_PAR_TBB)
TEST_CASE("Algorithm - TBB")
{
	SECTION("for_each") { test_for_each(execution::tbb_par); }
	SECTION("transform unary") { test_transform_unary(execution::tbb_par); }
	SECTION("transform binary") { test_transform_binary(execution::tbb_par); }
}
#endif

#if defined(UFO_PAR_OMP)
TEST_CASE("Algorithm - OpenMP")
{
	SECTION("for_each") { test_for_each(execution::omp_par); }
	SECTION("transform unary") { test_transform_unary(execution::omp_par); }
	SECTION("transform binary") { test_transform_binary(execution::omp_par); }
}
#endif

#if defined(UFO_PAR_GCD)
TEST_CASE("Algorithm - GCD")
{
	SECTION("for_each") { test_for_each(execution::gcd_par); }
	SECTION("transform unary") { test_transform_unary(execution::gcd_par); }
	SECTION("transform binary") { test_transform_binary(execution::gcd_par); }
}
#endif
