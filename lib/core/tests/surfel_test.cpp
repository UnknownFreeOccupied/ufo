// UFO
#include <ufo/core/surfel.hpp>

// STL
#include <array>
#include <cmath>
#include <sstream>
#include <vector>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static constexpr double kEps = 1e-4;

static bool approxEq(double a, double b, double eps = kEps)
{
	return std::abs(a - b) < eps;
}

// Shorthand: Vec3f from three floats.
static ufo::Vec3f v3(float x, float y, float z) { return {x, y, z}; }

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] Default construction")
{
	ufo::Surfel s;
	REQUIRE(s.empty());
	REQUIRE(s.numPoints() == 0);

	auto sum = s.sum();
	REQUIRE(sum[0] == Catch::Approx(0.0f));
	REQUIRE(sum[1] == Catch::Approx(0.0f));
	REQUIRE(sum[2] == Catch::Approx(0.0f));

	for (auto v : s.sumSquares()) {
		REQUIRE(v == Catch::Approx(0.0f));
	}
}

TEST_CASE("[core] [surfel] Single-point construction")
{
	ufo::Surfel s{v3(1.0f, 2.0f, 3.0f)};

	REQUIRE_FALSE(s.empty());
	REQUIRE(s.numPoints() == 1);

	auto sum = s.sum();
	REQUIRE(sum[0] == Catch::Approx(1.0f));
	REQUIRE(sum[1] == Catch::Approx(2.0f));
	REQUIRE(sum[2] == Catch::Approx(3.0f));

	// Single point has no variance — scatter matrix is zero.
	for (auto v : s.sumSquares()) {
		REQUIRE(v == Catch::Approx(0.0f));
	}

	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(1.0));
	REQUIRE(m[1] == Catch::Approx(2.0));
	REQUIRE(m[2] == Catch::Approx(3.0));
}

TEST_CASE("[core] [surfel] Initializer-list construction")
{
	ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f)}};

	REQUIRE(s.numPoints() == 2);

	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(0.0).margin(kEps));
	REQUIRE(m[1] == Catch::Approx(0.0).margin(kEps));
	REQUIRE(m[2] == Catch::Approx(0.0).margin(kEps));
}

TEST_CASE("[core] [surfel] Iterator-range construction")
{
	std::vector<ufo::Vec3f> pts = {v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f),
	                               v3(0.0f, 0.0f, 1.0f)};

	ufo::Surfel s{pts.begin(), pts.end()};

	REQUIRE(s.numPoints() == 3);

	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(1.0 / 3.0).margin(kEps));
	REQUIRE(m[1] == Catch::Approx(1.0 / 3.0).margin(kEps));
	REQUIRE(m[2] == Catch::Approx(1.0 / 3.0).margin(kEps));
}

TEST_CASE("[core] [surfel] Range construction")
{
	std::vector<ufo::Vec3f> pts = {v3(2.0f, 0.0f, 0.0f), v3(0.0f, 2.0f, 0.0f)};
	ufo::Surfel             s{pts};

	REQUIRE(s.numPoints() == 2);

	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(1.0).margin(kEps));
	REQUIRE(m[1] == Catch::Approx(1.0).margin(kEps));
	REQUIRE(m[2] == Catch::Approx(0.0).margin(kEps));
}

// ---------------------------------------------------------------------------
// Equality
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] Equality")
{
	ufo::Surfel a{{v3(1.0f, 2.0f, 3.0f)}};
	ufo::Surfel b{{v3(1.0f, 2.0f, 3.0f)}};
	ufo::Surfel c{{v3(4.0f, 5.0f, 6.0f)}};

	REQUIRE(a == b);
	REQUIRE_FALSE(a == c);
	REQUIRE(a != c);
	REQUIRE_FALSE(a != b);
}

// ---------------------------------------------------------------------------
// add / operator+= / operator+
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] add(Vec3f) accumulates points")
{
	ufo::Surfel s;
	s.add(v3(1.0f, 0.0f, 0.0f));
	REQUIRE(s.numPoints() == 1);

	s.add(v3(0.0f, 1.0f, 0.0f));
	REQUIRE(s.numPoints() == 2);

	s.add(v3(0.0f, 0.0f, 1.0f));
	REQUIRE(s.numPoints() == 3);

	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(1.0 / 3.0).margin(kEps));
	REQUIRE(m[1] == Catch::Approx(1.0 / 3.0).margin(kEps));
	REQUIRE(m[2] == Catch::Approx(1.0 / 3.0).margin(kEps));
}

TEST_CASE("[core] [surfel] add(Surfel) merges statistics correctly")
{
	// Build from individual points vs. merging two sub-surfels should agree.
	ufo::Surfel direct{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f), v3(0.0f, 0.0f, 1.0f),
	                    v3(1.0f, 1.0f, 0.0f)}};

	ufo::Surfel a{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f)}};
	ufo::Surfel b{{v3(0.0f, 0.0f, 1.0f), v3(1.0f, 1.0f, 0.0f)}};
	ufo::Surfel merged;
	merged.add(a);
	merged.add(b);

	REQUIRE(merged.numPoints() == direct.numPoints());

	auto md = direct.mean();
	auto mm = merged.mean();
	REQUIRE(mm[0] == Catch::Approx(md[0]).margin(kEps));
	REQUIRE(mm[1] == Catch::Approx(md[1]).margin(kEps));
	REQUIRE(mm[2] == Catch::Approx(md[2]).margin(kEps));

	auto ss_d = direct.sumSquares();
	auto ss_m = merged.sumSquares();
	for (std::size_t i = 0; i < 6; ++i) {
		REQUIRE(ss_m[i] == Catch::Approx(ss_d[i]).margin(kEps));
	}
}

TEST_CASE("[core] [surfel] add(Surfel) into empty copies directly")
{
	ufo::Surfel src{{v3(1.0f, 2.0f, 3.0f), v3(4.0f, 5.0f, 6.0f)}};
	ufo::Surfel dst;
	dst.add(src);
	REQUIRE(dst == src);
}

TEST_CASE("[core] [surfel] add(initializer_list)")
{
	ufo::Surfel s;
	s.add({v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f)});

	REQUIRE(s.numPoints() == 2);
	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(0.0).margin(kEps));
}

TEST_CASE("[core] [surfel] operator+= and operator+")
{
	SECTION("operator+=(Surfel)")
	{
		ufo::Surfel a{v3(1.0f, 0.0f, 0.0f)};
		ufo::Surfel b{v3(0.0f, 1.0f, 0.0f)};
		a += b;
		REQUIRE(a.numPoints() == 2);
		auto m = a.mean();
		REQUIRE(m[0] == Catch::Approx(0.5).margin(kEps));
		REQUIRE(m[1] == Catch::Approx(0.5).margin(kEps));
	}

	SECTION("operator+(Surfel, Surfel)")
	{
		ufo::Surfel a{v3(1.0f, 0.0f, 0.0f)};
		ufo::Surfel b{v3(0.0f, 1.0f, 0.0f)};
		ufo::Surfel c = a + b;
		REQUIRE(c.numPoints() == 2);
		auto m = c.mean();
		REQUIRE(m[0] == Catch::Approx(0.5).margin(kEps));
		REQUIRE(m[1] == Catch::Approx(0.5).margin(kEps));
	}

	SECTION("operator+=(Vec3f)")
	{
		ufo::Surfel s{v3(1.0f, 0.0f, 0.0f)};
		s += v3(-1.0f, 0.0f, 0.0f);
		REQUIRE(s.numPoints() == 2);
		auto m = s.mean();
		REQUIRE(m[0] == Catch::Approx(0.0).margin(kEps));
	}

	SECTION("operator+(Surfel, Vec3f)")
	{
		ufo::Surfel s{v3(1.0f, 0.0f, 0.0f)};
		ufo::Surfel t = s + v3(-1.0f, 0.0f, 0.0f);
		REQUIRE(t.numPoints() == 2);
		auto m = t.mean();
		REQUIRE(m[0] == Catch::Approx(0.0).margin(kEps));
	}
}

// ---------------------------------------------------------------------------
// remove / operator-= / operator-
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] remove(Vec3f) is inverse of add(Vec3f)")
{
	SECTION("Single point round-trip → empty")
	{
		ufo::Surfel s{v3(1.0f, 2.0f, 3.0f)};
		s.remove(v3(1.0f, 2.0f, 3.0f));
		REQUIRE(s.empty());
		REQUIRE(s.numPoints() == 0);
	}

	SECTION("Two points, remove one → back to single-point state")
	{
		ufo::Surfel ref{v3(1.0f, 2.0f, 3.0f)};

		ufo::Surfel s{v3(1.0f, 2.0f, 3.0f)};
		s.add(v3(4.0f, 5.0f, 6.0f));
		s.remove(v3(4.0f, 5.0f, 6.0f));

		REQUIRE(s.numPoints() == ref.numPoints());

		auto mr = ref.mean();
		auto ms = s.mean();
		REQUIRE(ms[0] == Catch::Approx(mr[0]).margin(kEps));
		REQUIRE(ms[1] == Catch::Approx(mr[1]).margin(kEps));
		REQUIRE(ms[2] == Catch::Approx(mr[2]).margin(kEps));

		auto ss_r = ref.sumSquares();
		auto ss_s = s.sumSquares();
		for (std::size_t i = 0; i < 6; ++i) {
			REQUIRE(ss_s[i] == Catch::Approx(ss_r[i]).margin(kEps));
		}
	}
}

TEST_CASE("[core] [surfel] remove(Surfel) is inverse of add(Surfel)")
{
	ufo::Surfel a{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f)}};
	ufo::Surfel b{{v3(0.0f, 0.0f, 1.0f), v3(1.0f, 1.0f, 0.0f)}};
	ufo::Surfel ref = a;

	ufo::Surfel merged = a + b;
	merged.remove(b);

	REQUIRE(merged.numPoints() == ref.numPoints());

	auto mm = merged.mean();
	auto mr = ref.mean();
	REQUIRE(mm[0] == Catch::Approx(mr[0]).margin(kEps));
	REQUIRE(mm[1] == Catch::Approx(mr[1]).margin(kEps));
	REQUIRE(mm[2] == Catch::Approx(mr[2]).margin(kEps));

	auto ss_m = merged.sumSquares();
	auto ss_r = ref.sumSquares();
	for (std::size_t i = 0; i < 6; ++i) {
		REQUIRE(ss_m[i] == Catch::Approx(ss_r[i]).margin(kEps));
	}
}

TEST_CASE("[core] [surfel] remove all points → empty")
{
	ufo::Surfel s{{v3(1.0f, 2.0f, 3.0f), v3(4.0f, 5.0f, 6.0f)}};
	s.remove(s);
	REQUIRE(s.empty());
}

TEST_CASE("[core] [surfel] operator-= and operator-")
{
	SECTION("operator-=(Surfel)")
	{
		ufo::Surfel a{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f)}};
		ufo::Surfel b{v3(0.0f, 1.0f, 0.0f)};
		ufo::Surfel ref{v3(1.0f, 0.0f, 0.0f)};
		a -= b;
		REQUIRE(a.numPoints() == ref.numPoints());
		auto m  = a.mean();
		auto mr = ref.mean();
		REQUIRE(m[0] == Catch::Approx(mr[0]).margin(kEps));
		REQUIRE(m[1] == Catch::Approx(mr[1]).margin(kEps));
		REQUIRE(m[2] == Catch::Approx(mr[2]).margin(kEps));
	}

	SECTION("operator-(Surfel, Surfel)")
	{
		ufo::Surfel a{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f)}};
		ufo::Surfel b{v3(0.0f, 1.0f, 0.0f)};
		ufo::Surfel ref{v3(1.0f, 0.0f, 0.0f)};
		ufo::Surfel c = a - b;
		REQUIRE(c.numPoints() == ref.numPoints());
		auto m  = c.mean();
		auto mr = ref.mean();
		REQUIRE(m[0] == Catch::Approx(mr[0]).margin(kEps));
	}

	SECTION("operator-=(Vec3f)")
	{
		ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f)}};
		s -= v3(0.0f, 1.0f, 0.0f);
		REQUIRE(s.numPoints() == 1);
	}

	SECTION("operator-(Surfel, Vec3f)")
	{
		ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f)}};
		ufo::Surfel t = s - v3(0.0f, 1.0f, 0.0f);
		REQUIRE(t.numPoints() == 1);
	}
}

TEST_CASE("[core] [surfel] remove(initializer_list)")
{
	ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f), v3(0.0f, 0.0f, 1.0f)}};
	s.remove({v3(0.0f, 1.0f, 0.0f), v3(0.0f, 0.0f, 1.0f)});

	REQUIRE(s.numPoints() == 1);
	auto m = s.mean();
	REQUIRE(m[0] == Catch::Approx(1.0).margin(kEps));
	REQUIRE(m[1] == Catch::Approx(0.0).margin(kEps));
	REQUIRE(m[2] == Catch::Approx(0.0).margin(kEps));
}

// ---------------------------------------------------------------------------
// clear
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] clear()")
{
	ufo::Surfel s{{v3(1.0f, 2.0f, 3.0f), v3(4.0f, 5.0f, 6.0f)}};
	REQUIRE_FALSE(s.empty());
	s.clear();
	REQUIRE(s.empty());
	REQUIRE(s.numPoints() == 0);
	for (auto val : s.sumSquares()) {
		REQUIRE(val == Catch::Approx(0.0f));
	}
}

// ---------------------------------------------------------------------------
// Statistical quantities
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] mean()")
{
	SECTION("Two symmetric points — mean at origin")
	{
		ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f)}};
		auto        m = s.mean();
		REQUIRE(m[0] == Catch::Approx(0.0).margin(kEps));
		REQUIRE(m[1] == Catch::Approx(0.0).margin(kEps));
		REQUIRE(m[2] == Catch::Approx(0.0).margin(kEps));
	}

	SECTION("Four points — mean is centroid")
	{
		// Points: (0,0,0), (4,0,0), (0,4,0), (4,4,0) → mean = (2,2,0)
		ufo::Surfel s{{v3(0.0f, 0.0f, 0.0f), v3(4.0f, 0.0f, 0.0f), v3(0.0f, 4.0f, 0.0f),
		               v3(4.0f, 4.0f, 0.0f)}};
		auto        m = s.mean();
		REQUIRE(m[0] == Catch::Approx(2.0).margin(kEps));
		REQUIRE(m[1] == Catch::Approx(2.0).margin(kEps));
		REQUIRE(m[2] == Catch::Approx(0.0).margin(kEps));
	}
}

TEST_CASE("[core] [surfel] covariance() — known distribution via sumSquares")
{
	// Points (1,0,0) and (-1,0,0): mean = (0,0,0).
	// Scatter matrix: Sxx = (1-0)^2 + (-1-0)^2 = 2, all others = 0.
	// Sample covariance = S / (n-1) = S / 1 = S.
	ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f)}};

	auto ss = s.sumSquares();
	// Order: Sxx, Sxy, Sxz, Syy, Syz, Szz
	REQUIRE(ss[0] == Catch::Approx(2.0f).margin(kEps));  // Sxx
	REQUIRE(ss[1] == Catch::Approx(0.0f).margin(kEps));  // Sxy
	REQUIRE(ss[2] == Catch::Approx(0.0f).margin(kEps));  // Sxz
	REQUIRE(ss[3] == Catch::Approx(0.0f).margin(kEps));  // Syy
	REQUIRE(ss[4] == Catch::Approx(0.0f).margin(kEps));  // Syz
	REQUIRE(ss[5] == Catch::Approx(0.0f).margin(kEps));  // Szz

	// Covariance matrix is symmetric by construction.
	auto C = s.covariance();
	REQUIRE(approxEq(C[0][1], C[1][0]));
	REQUIRE(approxEq(C[0][2], C[2][0]));
	REQUIRE(approxEq(C[1][2], C[2][1]));
}

TEST_CASE("[core] [surfel] normal() — perpendicular to point plane")
{
	// Four points in the z=0 plane. Normal must be (0, 0, ±1).
	ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f),
	               v3(0.0f, -1.0f, 0.0f)}};

	auto n = s.normal();
	REQUIRE(approxEq(std::abs(n[2]), 1.0));
	REQUIRE(approxEq(n[0], 0.0));
	REQUIRE(approxEq(n[1], 0.0));
}

TEST_CASE("[core] [surfel] planarity()")
{
	SECTION("Perfectly planar → planarity == 1")
	{
		ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f),
		               v3(0.0f, -1.0f, 0.0f)}};
		REQUIRE(s.planarity() == Catch::Approx(1.0).margin(kEps));
	}

	SECTION("Isotropic distribution → planarity == 0")
	{
		// Six points on unit axes — by symmetry covariance = (2/5)*I,
		// all eigenvalues equal, so planarity = 0.
		ufo::Surfel s{{v3(1.0f, 0.0f, 0.0f), v3(-1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f),
		               v3(0.0f, -1.0f, 0.0f), v3(0.0f, 0.0f, 1.0f), v3(0.0f, 0.0f, -1.0f)}};
		REQUIRE(s.planarity() == Catch::Approx(0.0).margin(kEps));
	}

	SECTION("Planarity is in [0, 1]")
	{
		ufo::Surfel s{{v3(1.0f, 0.5f, 0.1f), v3(-0.5f, 1.0f, 0.2f), v3(0.3f, -1.0f, 0.05f),
		               v3(0.1f, 0.2f, -0.3f)}};
		double      p = s.planarity();
		REQUIRE(p >= 0.0);
		REQUIRE(p <= 1.0);
	}
}

// ---------------------------------------------------------------------------
// Batch add/remove via iterator range
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] Batch add via iterator range matches incremental add")
{
	std::vector<ufo::Vec3f> pts = {v3(1.0f, 2.0f, 3.0f), v3(4.0f, 5.0f, 6.0f),
	                               v3(7.0f, 8.0f, 9.0f), v3(-1.0f, -2.0f, -3.0f)};

	ufo::Surfel batch{pts.begin(), pts.end()};

	ufo::Surfel incremental;
	for (auto const& p : pts) {
		incremental.add(p);
	}

	REQUIRE(batch.numPoints() == incremental.numPoints());

	auto mb = batch.mean();
	auto mi = incremental.mean();
	REQUIRE(mb[0] == Catch::Approx(mi[0]).margin(kEps));
	REQUIRE(mb[1] == Catch::Approx(mi[1]).margin(kEps));
	REQUIRE(mb[2] == Catch::Approx(mi[2]).margin(kEps));

	auto ssb = batch.sumSquares();
	auto ssi = incremental.sumSquares();
	for (std::size_t i = 0; i < 6; ++i) {
		REQUIRE(ssb[i] == Catch::Approx(ssi[i]).margin(kEps));
	}
}

TEST_CASE("[core] [surfel] Batch remove via iterator range")
{
	std::vector<ufo::Vec3f> all       = {v3(1.0f, 0.0f, 0.0f), v3(0.0f, 1.0f, 0.0f),
	                                     v3(0.0f, 0.0f, 1.0f)};
	std::vector<ufo::Vec3f> to_remove = {v3(0.0f, 1.0f, 0.0f), v3(0.0f, 0.0f, 1.0f)};

	ufo::Surfel s{all};
	s.remove(to_remove.begin(), to_remove.end());

	ufo::Surfel ref{v3(1.0f, 0.0f, 0.0f)};
	REQUIRE(s.numPoints() == ref.numPoints());

	auto m  = s.mean();
	auto mr = ref.mean();
	REQUIRE(m[0] == Catch::Approx(mr[0]).margin(kEps));
	REQUIRE(m[1] == Catch::Approx(mr[1]).margin(kEps));
	REQUIRE(m[2] == Catch::Approx(mr[2]).margin(kEps));
}

// ---------------------------------------------------------------------------
// Raw statistics accessors
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] numPoints(), sum(), sumSquares()")
{
	ufo::Surfel s{{v3(2.0f, 4.0f, 6.0f), v3(0.0f, 0.0f, 0.0f)}};

	REQUIRE(s.numPoints() == 2);

	auto sum = s.sum();
	REQUIRE(sum[0] == Catch::Approx(2.0f));
	REQUIRE(sum[1] == Catch::Approx(4.0f));
	REQUIRE(sum[2] == Catch::Approx(6.0f));

	// mean_x = 1, deviations = (2-1)=1 and (0-1)=-1, Sxx = 1 + 1 = 2
	auto ss = s.sumSquares();
	REQUIRE(ss[0] == Catch::Approx(2.0f).margin(kEps));
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] std::format")
{
	SECTION("Empty surfel")
	{
		ufo::Surfel s;
		REQUIRE(std::format("{}", s) == "Surfel{empty}");
	}

	SECTION("Single point at origin")
	{
		ufo::Surfel s{v3(0.0f, 0.0f, 0.0f)};
		REQUIRE(std::format("{}", s) == "Surfel{n=1, mean=(0, 0, 0)}");
	}

	SECTION("Single point at (1, 2, 3)")
	{
		ufo::Surfel s{v3(1.0f, 2.0f, 3.0f)};
		REQUIRE(std::format("{}", s) == "Surfel{n=1, mean=(1, 2, 3)}");
	}
}

TEST_CASE("[core] [surfel] operator<<")
{
	SECTION("Empty surfel")
	{
		std::ostringstream oss;
		oss << ufo::Surfel{};
		REQUIRE(oss.str() == "Surfel{empty}");
	}

	SECTION("Non-empty surfel")
	{
		std::ostringstream oss;
		oss << ufo::Surfel{v3(0.0f, 0.0f, 0.0f)};
		REQUIRE(oss.str() == "Surfel{n=1, mean=(0, 0, 0)}");
	}
}

// ---------------------------------------------------------------------------
// Numerical stability
// ---------------------------------------------------------------------------

TEST_CASE("[core] [surfel] Add/remove round-trip with many points is stable")
{
	std::vector<ufo::Vec3f> pts;
	for (int i = 0; i < 20; ++i) {
		pts.push_back(
		    v3(static_cast<float>(i), static_cast<float>(i * 2), static_cast<float>(i * 3)));
	}

	ufo::Surfel s{pts};
	for (auto const& p : pts) {
		s.remove(p);
	}
	REQUIRE(s.empty());
}

TEST_CASE("[core] [surfel] Merge order is commutative for mean")
{
	ufo::Surfel a{{v3(1.0f, 0.0f, 0.0f), v3(2.0f, 0.0f, 0.0f)}};
	ufo::Surfel b{{v3(3.0f, 0.0f, 0.0f), v3(4.0f, 0.0f, 0.0f)}};

	ufo::Surfel ab = a + b;
	ufo::Surfel ba = b + a;

	auto mab = ab.mean();
	auto mba = ba.mean();
	REQUIRE(mab[0] == Catch::Approx(mba[0]).margin(kEps));
	REQUIRE(mab[1] == Catch::Approx(mba[1]).margin(kEps));
	REQUIRE(mab[2] == Catch::Approx(mba[2]).margin(kEps));
}
