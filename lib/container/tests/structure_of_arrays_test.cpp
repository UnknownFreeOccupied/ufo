// UFO
#include <ufo/container/structure_of_arrays.hpp>

// Catch2
#include <catch2/catch_test_macros.hpp>

// STL
#include <string>

using namespace ufo;

TEST_CASE("StructureOfArrays Erase", "[container][soa]")
{
	SoA<int, float, std::string> soa;
	soa.push_back({1, 1.0f, "one"});
	soa.push_back({2, 2.0f, "two"});
	soa.push_back({3, 3.0f, "three"});
	soa.push_back({2, 2.0f, "two"});

	SECTION("erase value")
	{
		auto removed = erase(soa, std::make_tuple(2, 2.0f, std::string("two")));
		CHECK(removed == 2);
		CHECK(soa.size() == 2);
		CHECK(std::get<0>(soa[0]) == 1);
		CHECK(std::get<0>(soa[1]) == 3);
	}

	SECTION("erase_if predicate")
	{
		auto removed = erase_if(soa, [](auto const& row) { return std::get<0>(row) >= 2; });
		CHECK(removed == 3);
		CHECK(soa.size() == 1);
		CHECK(std::get<0>(soa[0]) == 1);
	}
}

TEST_CASE("StructureOfArrays Comparison", "[container][soa]")
{
	SoA<int, float> soa1;
	soa1.push_back({1, 1.0f});
	soa1.push_back({2, 2.0f});

	SoA<int, float> soa2;
	soa2.push_back({1, 1.0f});
	soa2.push_back({2, 2.0f});

	SoA<int, float> soa3;
	soa3.push_back({1, 1.0f});
	soa3.push_back({3, 3.0f});

	SECTION("operator==")
	{
		CHECK(bool(soa1 == soa2));
		CHECK_FALSE(bool(soa1 == soa3));
	}

	SECTION("operator<=>")
	{
		CHECK(bool((soa1 <=> soa2) == std::strong_ordering::equal));
		CHECK(bool((soa1 <=> soa3) == std::strong_ordering::less));
		CHECK(bool((soa3 <=> soa1) == std::strong_ordering::greater));

		// Derived comparisons
		CHECK(bool(soa1 <= soa2));
		CHECK(bool(soa1 >= soa2));
		CHECK(bool(soa1 < soa3));
		CHECK(bool(soa3 > soa1));
	}
}

TEST_CASE("StructureOfArrays Duplicate Types", "[container][soa]")
{
	SoA<int, int> soa;
	soa.push_back({1, 2});
	soa.push_back({3, 4});

	CHECK(soa.size() == 2);
	CHECK(std::get<0>(soa[0]) == 1);
	CHECK(std::get<1>(soa[0]) == 2);
	CHECK(std::get<0>(soa[1]) == 3);
	CHECK(std::get<1>(soa[1]) == 4);
}

TEST_CASE("StructureOfArrays Move Semantics", "[container][soa]")
{
	SoA<int, float> soa1;
	soa1.push_back({1, 1.0f});
	soa1.push_back({2, 2.0f});

	SoA<int, float> soa2 = std::move(soa1);

	CHECK(soa2.size() == 2);
	CHECK(std::get<0>(soa2[0]) == 1);
	CHECK(std::get<1>(soa2[0]) == 1.0f);

	// Verify view is valid after move
	auto it = soa2.begin();
	CHECK(std::get<0>(*it) == 1);
}

TEST_CASE("StructureOfArrays Emplace", "[container][soa]")
{
	SoA<int, std::string> soa;
	soa.emplace_back(1, "one");
	soa.emplace(soa.begin(), 0, "zero");

	CHECK(soa.size() == 2);
	CHECK(std::get<0>(soa[0]) == 0);
	CHECK(std::get<1>(soa[0]) == "zero");
	CHECK(std::get<0>(soa[1]) == 1);
	CHECK(std::get<1>(soa[1]) == "one");
}

TEST_CASE("StructureOfArrays Bounds Check", "[container][soa]")
{
	SoA<int> soa;
	soa.push_back({1});

	CHECK_NOTHROW(soa.at(0));
	CHECK_THROWS_AS(soa.at(1), std::out_of_range);
}
TEST_CASE("StructureOfArrays Default Construction", "[container][soa]")
{
	SoA<int, float> soa;
	CHECK(soa.empty());
	CHECK(soa.size() == 0);
	CHECK(soa.begin() == soa.end());
}

TEST_CASE("StructureOfArrays Refinements", "[container][soa]")
{
	SECTION("Deduction Guide")
	{
		SoA soa1 = {std::make_tuple(1, 1.0f), std::make_tuple(2, 2.0f)};
		static_assert(std::is_same_v<decltype(soa1), SoA<int, float>>);
		CHECK(soa1.size() == 2);

		SoA soa2(10, std::make_tuple(1, 1.0f));
		static_assert(std::is_same_v<decltype(soa2), SoA<int, float>>);
		CHECK(soa2.size() == 10);
	}

	SECTION("get<I>(pos)")
	{
		SoA<int, float> soa;
		soa.push_back({1, 1.0f});
		CHECK(soa.get<0>(0) == 1);
		CHECK(soa.get<1>(0) == 1.0f);
		soa.get<0>(0) = 10;
		CHECK(std::get<0>(soa[0]) == 10);
	}

	SECTION("constexpr")
	{
		constexpr auto size = []() {
			SoA<int, float> soa;
			return soa.size();
		}();
		static_assert(size == 0);
	}

	SECTION("From Components")
	{
		std::vector<int>   vec_i = {1, 2};
		std::vector<float> vec_f = {1.0f, 2.0f};
		SoA                soa(std::move(vec_i), std::move(vec_f));
		static_assert(std::is_same_v<decltype(soa), SoA<int, float>>);
		CHECK(soa.size() == 2);
		CHECK(soa.get<0>(0) == 1);
		CHECK(soa.get<1>(1) == 2.0f);
	}

	SECTION("Variadic push_back")
	{
		SoA<int, float, std::string> soa;
		soa.push_back(1, 1.0f, "one");
		CHECK(soa.size() == 1);
		CHECK(soa.get<0>(0) == 1);
		CHECK(soa.get<1>(0) == 1.0f);
		CHECK(soa.get<2>(0) == "one");
	}

	SECTION("Type-based get<T>(pos)")
	{
		SoA<int, float> soa;
		soa.push_back(1, 1.0f);
		CHECK(soa.get<int>(0) == 1);
		CHECK(soa.get<float>(0) == 1.0f);
		soa.get<int>(0) = 10;
		CHECK(soa.get<0>(0) == 10);
	}

	SECTION("Non-member swap")
	{
		SoA<int> soa1, soa2;
		soa1.push_back(1);
		soa2.push_back(2);
		swap(soa1, soa2);
		CHECK(soa1.get<0>(0) == 2);
		CHECK(soa2.get<0>(0) == 1);
	}

	SECTION("Merge")
	{
		SoA<int, float> a;
		a.push_back(1, 1.1f);
		a.push_back(2, 2.2f);

		SoA<std::string> b;
		b.push_back("one");
		b.push_back("two");

		// Merge two SoA
		SoA c(a, b);
		static_assert(std::is_same_v<decltype(c), SoA<int, float, std::string>>);
		CHECK(c.size() == 2);
		CHECK(c.get<0>(0) == 1);
		CHECK(c.get<2>(1) == "two");

		// Merge SoA and vector
		std::vector<double> d_vec = {10.0, 20.0};
		SoA                 d(d_vec, a, b);
		static_assert(std::is_same_v<decltype(d), SoA<double, int, float, std::string>>);
		CHECK(d.size() == 2);
		CHECK(d.get<0>(1) == 20.0);
		CHECK(d.get<1>(1) == 2);

		// Throw on size mismatch
		std::vector<int> e_vec = {1, 2, 3};
		CHECK_THROWS_AS((SoA<int, int>(e_vec, a.view<int>())), std::invalid_argument);
	}
}
