// UFO
#include <ufo/core/range.hpp>

// STL
#include <set>
#include <sstream>
#include <type_traits>

// Catch2
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[core] [range] Aggregate properties")
{
	STATIC_REQUIRE(std::is_aggregate_v<ufo::Range<int>>);
	STATIC_REQUIRE(std::is_trivially_copyable_v<ufo::Range<int>>);
	STATIC_REQUIRE(std::is_trivially_copyable_v<ufo::Range<float>>);
	STATIC_REQUIRE(std::is_same_v<ufo::Range<int>::value_type, int>);
	STATIC_REQUIRE(std::is_same_v<ufo::Range<double>::value_type, double>);
	STATIC_REQUIRE(ufo::Range<int>::Comparator::is_transparent::value);
}

TEST_CASE("[core] [range] Default initialization")
{
	ufo::Range<int>    ri{};
	ufo::Range<float>  rf{};
	ufo::Range<double> rd{};
	REQUIRE(ri.lower == 0);
	REQUIRE(ri.upper == 0);
	REQUIRE(rf.lower == 0.0f);
	REQUIRE(rf.upper == 0.0f);
	REQUIRE(rd.lower == 0.0);
	REQUIRE(rd.upper == 0.0);
}

TEST_CASE("[core] [range] Aggregate initialization")
{
	SECTION("Two-argument")
	{
		ufo::Range<int> r{1, 5};
		REQUIRE(r.lower == 1);
		REQUIRE(r.upper == 5);
	}

	SECTION("Degenerate (lower == upper)")
	{
		ufo::Range<int> r{3, 3};
		REQUIRE(r.lower == 3);
		REQUIRE(r.upper == 3);
	}

	SECTION("Negative bounds")
	{
		ufo::Range<int> r{-10, -2};
		REQUIRE(r.lower == -10);
		REQUIRE(r.upper == -2);
	}

	SECTION("Float bounds")
	{
		ufo::Range<float> r{0.5f, 1.5f};
		REQUIRE(r.lower == 0.5f);
		REQUIRE(r.upper == 1.5f);
	}

	SECTION("Unsigned bounds")
	{
		ufo::Range<unsigned> r{2u, 8u};
		REQUIRE(r.lower == 2u);
		REQUIRE(r.upper == 8u);
	}
}

TEST_CASE("[core] [range] contains(T)")
{
	ufo::Range<int> r{1, 10};

	SECTION("Interior values")
	{
		REQUIRE(r.contains(5));
		REQUIRE(r.contains(2));
		REQUIRE(r.contains(9));
	}

	SECTION("Boundary values")
	{
		REQUIRE(r.contains(1));
		REQUIRE(r.contains(10));
	}

	SECTION("Outside values")
	{
		REQUIRE_FALSE(r.contains(0));
		REQUIRE_FALSE(r.contains(11));
		REQUIRE_FALSE(r.contains(-100));
		REQUIRE_FALSE(r.contains(100));
	}

	SECTION("Degenerate range")
	{
		ufo::Range<int> d{5, 5};
		REQUIRE(d.contains(5));
		REQUIRE_FALSE(d.contains(4));
		REQUIRE_FALSE(d.contains(6));
	}

	SECTION("Float range")
	{
		ufo::Range<float> rf{0.0f, 1.0f};
		REQUIRE(rf.contains(0.0f));
		REQUIRE(rf.contains(0.5f));
		REQUIRE(rf.contains(1.0f));
		REQUIRE_FALSE(rf.contains(-0.001f));
		REQUIRE_FALSE(rf.contains(1.001f));
	}
}

TEST_CASE("[core] [range] contains(Range)")
{
	ufo::Range<int> r{1, 10};

	SECTION("Same range") { REQUIRE(r.contains(ufo::Range<int>{1, 10})); }

	SECTION("Strictly interior") { REQUIRE(r.contains(ufo::Range<int>{3, 7})); }

	SECTION("Touching lower bound") { REQUIRE(r.contains(ufo::Range<int>{1, 5})); }

	SECTION("Touching upper bound") { REQUIRE(r.contains(ufo::Range<int>{5, 10})); }

	SECTION("Partially outside — lower")
	{
		REQUIRE_FALSE(r.contains(ufo::Range<int>{0, 5}));
	}

	SECTION("Partially outside — upper")
	{
		REQUIRE_FALSE(r.contains(ufo::Range<int>{5, 11}));
	}

	SECTION("Completely outside — before")
	{
		REQUIRE_FALSE(r.contains(ufo::Range<int>{-5, -1}));
	}

	SECTION("Completely outside — after")
	{
		REQUIRE_FALSE(r.contains(ufo::Range<int>{11, 15}));
	}

	SECTION("Degenerate argument")
	{
		REQUIRE(r.contains(ufo::Range<int>{5, 5}));
		REQUIRE_FALSE(r.contains(ufo::Range<int>{0, 0}));
		REQUIRE_FALSE(r.contains(ufo::Range<int>{11, 11}));
	}
}

TEST_CASE("[core] [range] operator== and operator!=")
{
	ufo::Range<int> r1{1, 5};
	ufo::Range<int> r2{1, 5};
	ufo::Range<int> r3{1, 6};
	ufo::Range<int> r4{2, 5};

	REQUIRE(r1 == r2);
	REQUIRE_FALSE(r1 == r3);
	REQUIRE_FALSE(r1 == r4);
	REQUIRE(r1 != r3);
	REQUIRE(r1 != r4);
	REQUIRE_FALSE(r1 != r2);
}

TEST_CASE("[core] [range] Range-Range interval ordering")
{
	// These operators implement interval ordering, NOT value ordering.

	SECTION("operator< — entirely before")
	{
		REQUIRE(ufo::Range<int>{1, 3} < ufo::Range<int>{4, 6});        // 3 < 4
		REQUIRE_FALSE(ufo::Range<int>{1, 3} < ufo::Range<int>{3, 6});  // 3 < 3 false
		REQUIRE_FALSE(ufo::Range<int>{4, 6} < ufo::Range<int>{1, 3});
	}

	SECTION("operator<= — upper <= rhs.lower")
	{
		REQUIRE(ufo::Range<int>{1, 3} <= ufo::Range<int>{3, 6});        // 3 <= 3 true
		REQUIRE(ufo::Range<int>{1, 3} <= ufo::Range<int>{4, 6});        // 3 <= 4 true
		REQUIRE_FALSE(ufo::Range<int>{1, 4} <= ufo::Range<int>{3, 6});  // 4 <= 3 false
	}

	SECTION("operator> — entirely after")
	{
		REQUIRE(ufo::Range<int>{4, 6} > ufo::Range<int>{1, 3});        // 4 > 3
		REQUIRE_FALSE(ufo::Range<int>{3, 6} > ufo::Range<int>{1, 3});  // 3 > 3 false
		REQUIRE_FALSE(ufo::Range<int>{1, 3} > ufo::Range<int>{4, 6});
	}

	SECTION("operator>= — lower >= rhs.upper")
	{
		REQUIRE(ufo::Range<int>{3, 6} >= ufo::Range<int>{1, 3});        // 3 >= 3 true
		REQUIRE(ufo::Range<int>{4, 6} >= ufo::Range<int>{1, 3});        // 4 >= 3 true
		REQUIRE_FALSE(ufo::Range<int>{2, 6} >= ufo::Range<int>{1, 3});  // 2 >= 3 false
	}
}

TEST_CASE("[core] [range] Range-scalar ordering")
{
	SECTION("Range < T — upper < rhs")
	{
		REQUIRE(ufo::Range<int>{1, 5} < 6);
		REQUIRE(ufo::Range<int>{1, 5} < 100);
		REQUIRE_FALSE(ufo::Range<int>{1, 5} < 5);  // 5 < 5 false
		REQUIRE_FALSE(ufo::Range<int>{1, 5} < 3);
	}

	SECTION("Range <= T — upper <= rhs")
	{
		REQUIRE(ufo::Range<int>{1, 5} <= 5);
		REQUIRE(ufo::Range<int>{1, 5} <= 6);
		REQUIRE_FALSE(ufo::Range<int>{1, 5} <= 4);
	}

	SECTION("Range > T — lower > rhs")
	{
		REQUIRE(ufo::Range<int>{5, 10} > 4);
		REQUIRE(ufo::Range<int>{5, 10} > 0);
		REQUIRE_FALSE(ufo::Range<int>{5, 10} > 5);  // 5 > 5 false
		REQUIRE_FALSE(ufo::Range<int>{5, 10} > 7);
	}

	SECTION("Range >= T — lower >= rhs")
	{
		REQUIRE(ufo::Range<int>{5, 10} >= 5);
		REQUIRE(ufo::Range<int>{5, 10} >= 4);
		REQUIRE_FALSE(ufo::Range<int>{5, 10} >= 6);
	}
}

TEST_CASE("[core] [range] Scalar-Range ordering")
{
	SECTION("T < Range — lhs < rhs.lower")
	{
		REQUIRE(3 < ufo::Range<int>{5, 10});
		REQUIRE_FALSE(5 < ufo::Range<int>{5, 10});  // 5 < 5 false
		REQUIRE_FALSE(7 < ufo::Range<int>{5, 10});
	}

	SECTION("T <= Range — lhs <= rhs.lower")
	{
		REQUIRE(5 <= ufo::Range<int>{5, 10});
		REQUIRE(4 <= ufo::Range<int>{5, 10});
		REQUIRE_FALSE(6 <= ufo::Range<int>{5, 10});
	}

	SECTION("T > Range — lhs > rhs.upper")
	{
		REQUIRE(11 > ufo::Range<int>{5, 10});
		REQUIRE_FALSE(10 > ufo::Range<int>{5, 10});  // 10 > 10 false
		REQUIRE_FALSE(7 > ufo::Range<int>{5, 10});
	}

	SECTION("T >= Range — lhs >= rhs.upper")
	{
		REQUIRE(10 >= ufo::Range<int>{5, 10});
		REQUIRE(11 >= ufo::Range<int>{5, 10});
		REQUIRE_FALSE(9 >= ufo::Range<int>{5, 10});
	}
}

TEST_CASE("[core] [range] Comparator in std::set")
{
	std::set<ufo::Range<int>, ufo::Range<int>::Comparator> s;
	s.insert({1, 3});
	s.insert({5, 8});
	s.insert({10, 15});

	REQUIRE(s.size() == 3);

	SECTION("Find by Range value")
	{
		auto it = s.find(ufo::Range<int>{5, 8});
		REQUIRE(it != s.end());
		REQUIRE(it->lower == 5);
		REQUIRE(it->upper == 8);
	}

	SECTION("Find by pair (heterogeneous lookup)")
	{
		auto it = s.find(std::pair<int, int>{5, 8});
		REQUIRE(it != s.end());
		REQUIRE(it->lower == 5);
		REQUIRE(it->upper == 8);
	}

	SECTION("Find absent range") { REQUIRE(s.find(ufo::Range<int>{4, 6}) == s.end()); }

	SECTION("Iteration order follows interval ordering")
	{
		auto it = s.begin();
		REQUIRE(it->lower == 1);
		++it;
		REQUIRE(it->lower == 5);
		++it;
		REQUIRE(it->lower == 10);
	}
}

TEST_CASE("[core] [range] std::format")
{
	SECTION("Integral — degenerate")
	{
		REQUIRE(std::format("{}", ufo::Range<int>{5, 5}) == "[5]");
	}

	SECTION("Integral — range")
	{
		REQUIRE(std::format("{}", ufo::Range<int>{1, 5}) == "[1..5]");
	}

	SECTION("Integral — negative range")
	{
		REQUIRE(std::format("{}", ufo::Range<int>{-3, 2}) == "[-3..2]");
	}

	SECTION("Float — degenerate")
	{
		REQUIRE(std::format("{}", ufo::Range<float>{3.0f, 3.0f}) == "[3]");
	}

	SECTION("Float — range")
	{
		REQUIRE(std::format("{}", ufo::Range<float>{1.5f, 2.5f}) == "[1.5,2.5]");
	}

	SECTION("Char — promoted to int, not printed as character")
	{
		ufo::Range<char> r{65, 65};  // ASCII 'A'
		REQUIRE(std::format("{}", r) == "[65]");
	}

	SECTION("Unsigned int")
	{
		REQUIRE(std::format("{}", ufo::Range<unsigned>{0u, 100u}) == "[0..100]");
	}
}

TEST_CASE("[core] [range] operator<<")
{
	SECTION("Integral range")
	{
		std::ostringstream oss;
		oss << ufo::Range<int>{1, 5};
		REQUIRE(oss.str() == "[1..5]");
	}

	SECTION("Degenerate")
	{
		std::ostringstream oss;
		oss << ufo::Range<int>{7, 7};
		REQUIRE(oss.str() == "[7]");
	}

	SECTION("Float range")
	{
		std::ostringstream oss;
		oss << ufo::Range<float>{0.5f, 1.5f};
		REQUIRE(oss.str() == "[0.5,1.5]");
	}
}
