// UFO
#include <ufo/numeric/vec.hpp>

// STL
#include <cmath>
#include <limits>
#include <numbers>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[Vec3d] [operator==] Equality operator")
{
	ufo::Vec3d v1(1.0, 2.0, 3.0);
	ufo::Vec3d v2(1.0, 2.0, 3.0);
	REQUIRE(v1 == v2);
}

TEST_CASE("[Vec3d] [operator!=] Inequality operator")
{
	ufo::Vec3d v1(1.0, 2.0, 3.0);
	ufo::Vec3d v2(1.0, 2.0, 4.0);
	REQUIRE(v1 != v2);
}

TEST_CASE("[Vec3d] [operator+] Addition operator")
{
	ufo::Vec3d v1(1.0, 2.0, 3.0);
	ufo::Vec3d v2(4.0, 5.0, 6.0);
	double     scalar = 2.0;

	SECTION("vec + vec")
	{
		ufo::Vec3d r = v1 + v2;
		REQUIRE(r[0] == 5.0);
		REQUIRE(r[1] == 7.0);
		REQUIRE(r[2] == 9.0);
	}

	SECTION("vec + scalar")
	{
		ufo::Vec3d r = v1 + scalar;
		REQUIRE(r[0] == 3.0);
		REQUIRE(r[1] == 4.0);
		REQUIRE(r[2] == 5.0);
	}

	SECTION("scalar + vec")
	{
		ufo::Vec3d r = scalar + v1;
		REQUIRE(r[0] == 3.0);
		REQUIRE(r[1] == 4.0);
		REQUIRE(r[2] == 5.0);
	}
}

TEST_CASE("[Vec3d] [operator-] Subtraction operator")
{
	ufo::Vec3d v1(1.0, 2.0, 3.0);
	ufo::Vec3d v2(4.0, 5.0, 6.0);
	double     scalar = 1.0;

	SECTION("vec - vec")
	{
		ufo::Vec3d r = v1 - v2;
		REQUIRE(r[0] == -3.0);
		REQUIRE(r[1] == -3.0);
		REQUIRE(r[2] == -3.0);
	}

	SECTION("vec - scalar")
	{
		ufo::Vec3d r = v1 - scalar;
		REQUIRE(r[0] == 0.0);
		REQUIRE(r[1] == 1.0);
		REQUIRE(r[2] == 2.0);
	}

	SECTION("scalar - vec")
	{
		ufo::Vec3d r = scalar - v1;
		REQUIRE(r[0] == 0.0);
		REQUIRE(r[1] == -1.0);
		REQUIRE(r[2] == -2.0);
	}

	SECTION("Unary negation")
	{
		ufo::Vec3d r = -v1;
		REQUIRE(r[0] == -1.0);
		REQUIRE(r[1] == -2.0);
		REQUIRE(r[2] == -3.0);
	}
}

TEST_CASE("[Vec3d] [operator*] Multiplication operator")
{
	ufo::Vec3d v(1.0, 2.0, 3.0);
	double     scalar = 2.0;

	SECTION("vec * scalar")
	{
		ufo::Vec3d r = v * scalar;
		REQUIRE(r[0] == 2.0);
		REQUIRE(r[1] == 4.0);
		REQUIRE(r[2] == 6.0);
	}

	SECTION("scalar * vec")
	{
		ufo::Vec3d r = scalar * v;
		REQUIRE(r[0] == 2.0);
		REQUIRE(r[1] == 4.0);
		REQUIRE(r[2] == 6.0);
	}

	SECTION("vec * vec (component-wise)")
	{
		ufo::Vec3d v2(2.0, 3.0, 4.0);
		ufo::Vec3d r = v * v2;
		REQUIRE(r[0] == 2.0);
		REQUIRE(r[1] == 6.0);
		REQUIRE(r[2] == 12.0);
	}

	SECTION("zero vector * scalar")
	{
		ufo::Vec3d zero(0.0, 0.0, 0.0);
		ufo::Vec3d r = zero * scalar;
		REQUIRE(r[0] == 0.0);
		REQUIRE(r[1] == 0.0);
		REQUIRE(r[2] == 0.0);
	}

	SECTION("vec * negative scalar")
	{
		ufo::Vec3d r = v * -2.0;
		REQUIRE(r[0] == -2.0);
		REQUIRE(r[1] == -4.0);
		REQUIRE(r[2] == -6.0);
	}
}

TEST_CASE("[Vec3d] [operator/] Division operator")
{
	ufo::Vec3d v1(4.0, 6.0, 8.0);
	ufo::Vec3d v2(2.0, 3.0, 4.0);
	double     scalar = 2.0;

	SECTION("vec / vec (component-wise)")
	{
		ufo::Vec3d r = v1 / v2;
		REQUIRE(r[0] == 2.0);
		REQUIRE(r[1] == 2.0);
		REQUIRE(r[2] == 2.0);
	}

	SECTION("vec / scalar")
	{
		ufo::Vec3d r = v1 / scalar;
		REQUIRE(r[0] == 2.0);
		REQUIRE(r[1] == 3.0);
		REQUIRE(r[2] == 4.0);
	}

	SECTION("scalar / vec")
	{
		ufo::Vec3d r = scalar / v2;
		REQUIRE(r[0] == 1.0);
		REQUIRE(r[1] == Catch::Approx(2.0 / 3.0));
		REQUIRE(r[2] == 0.5);
	}

	SECTION("vec / zero scalar yields infinity")
	{
		ufo::Vec3d r = v1 / 0.0;
		REQUIRE(std::isinf(r[0]));
		REQUIRE(std::isinf(r[1]));
		REQUIRE(std::isinf(r[2]));
	}
}

TEST_CASE("[Vec3d] [operator+=] Compound addition operator")
{
	ufo::Vec3d v(1.0, 2.0, 3.0);
	ufo::Vec3d other(4.0, 5.0, 6.0);
	double     scalar = 10.0;

	SECTION("vec += vec")
	{
		v += other;
		REQUIRE(v[0] == 5.0);
		REQUIRE(v[1] == 7.0);
		REQUIRE(v[2] == 9.0);
	}

	SECTION("vec += scalar")
	{
		v += scalar;
		REQUIRE(v[0] == 11.0);
		REQUIRE(v[1] == 12.0);
		REQUIRE(v[2] == 13.0);
	}
}

TEST_CASE("[Vec3d] [operator-=] Compound subtraction operator")
{
	ufo::Vec3d v(5.0, 7.0, 9.0);
	ufo::Vec3d other(1.0, 2.0, 3.0);
	double     scalar = 2.0;

	SECTION("vec -= vec")
	{
		v -= other;
		REQUIRE(v[0] == 4.0);
		REQUIRE(v[1] == 5.0);
		REQUIRE(v[2] == 6.0);
	}

	SECTION("vec -= scalar")
	{
		v -= scalar;
		REQUIRE(v[0] == 3.0);
		REQUIRE(v[1] == 5.0);
		REQUIRE(v[2] == 7.0);
	}
}

TEST_CASE("[Vec3d] [operator*=] Compound multiplication operator")
{
	ufo::Vec3d v(1.0, 2.0, 3.0);
	ufo::Vec3d other(2.0, 3.0, 4.0);
	double     scalar = 2.0;

	SECTION("vec *= vec")
	{
		v *= other;
		REQUIRE(v[0] == 2.0);
		REQUIRE(v[1] == 6.0);
		REQUIRE(v[2] == 12.0);
	}

	SECTION("vec *= scalar")
	{
		v *= scalar;
		REQUIRE(v[0] == 2.0);
		REQUIRE(v[1] == 4.0);
		REQUIRE(v[2] == 6.0);
	}
}

TEST_CASE("[Vec3d] [operator/=] Compound division operator")
{
	ufo::Vec3d v(6.0, 9.0, 12.0);
	ufo::Vec3d other(2.0, 3.0, 4.0);
	double     scalar = 3.0;

	SECTION("vec /= vec")
	{
		v /= other;
		REQUIRE(v[0] == 3.0);
		REQUIRE(v[1] == 3.0);
		REQUIRE(v[2] == 3.0);
	}

	SECTION("vec /= scalar")
	{
		v /= scalar;
		REQUIRE(v[0] == 2.0);
		REQUIRE(v[1] == 3.0);
		REQUIRE(v[2] == 4.0);
	}
}

TEST_CASE("[Vec3d] [operator[]] Subscript operator")
{
	ufo::Vec3d v(1.5, 2.5, 3.5);
	REQUIRE(v[0] == 1.5);
	REQUIRE(v[1] == 2.5);
	REQUIRE(v[2] == 3.5);

	ufo::Vec3d const cv(4.0, 5.0, 6.0);
	REQUIRE(cv[0] == 4.0);
	REQUIRE(cv[1] == 5.0);
	REQUIRE(cv[2] == 6.0);
}

TEST_CASE("[Vec3d] [size]")
{
	ufo::Vec3d v(1.0, 2.0, 3.0);
	REQUIRE(v.size() == 3);
}

TEST_CASE("[Vec3d] [dot] Dot product")
{
	SECTION("Dot product of zero vectors")
	{
		ufo::Vec3d v1(0.0, 0.0, 0.0);
		ufo::Vec3d v2(0.0, 0.0, 0.0);
		REQUIRE(dot(v1, v2) == Catch::Approx(0.0));
	}

	SECTION("Dot product with one zero vector")
	{
		ufo::Vec3d v1(1.0, 2.0, 3.0);
		ufo::Vec3d v2(0.0, 0.0, 0.0);
		REQUIRE(dot(v1, v2) == Catch::Approx(0.0));
	}

	SECTION("Dot product of orthogonal unit vectors")
	{
		ufo::Vec3d v1(1.0, 0.0, 0.0);
		ufo::Vec3d v2(0.0, 1.0, 0.0);
		REQUIRE(dot(v1, v2) == Catch::Approx(0.0));
	}

	SECTION("Dot product of parallel vectors")
	{
		ufo::Vec3d v1(1.0, 1.0, 1.0);
		ufo::Vec3d v2(2.0, 2.0, 2.0);
		REQUIRE(dot(v1, v2) == Catch::Approx(6.0));
	}

	SECTION("Dot product with negative components")
	{
		ufo::Vec3d v1(1.0, -1.0, 0.0);
		ufo::Vec3d v2(-1.0, 1.0, 0.0);
		REQUIRE(dot(v1, v2) == Catch::Approx(-2.0));
	}

	SECTION("Dot product of arbitrary vectors")
	{
		ufo::Vec3d v1(1.0, 2.0, 3.0);
		ufo::Vec3d v2(4.0, 5.0, 6.0);
		REQUIRE(dot(v1, v2) == Catch::Approx(32.0));
	}
}

TEST_CASE("[Vec3d] [cross] Cross product")
{
	SECTION("Cross product of zero vectors")
	{
		ufo::Vec3d v1(0.0, 0.0, 0.0);
		ufo::Vec3d v2(0.0, 0.0, 0.0);
		auto       r = cross(v1, v2);
		REQUIRE(r[0] == 0.0);
		REQUIRE(r[1] == 0.0);
		REQUIRE(r[2] == 0.0);
	}

	SECTION("x-axis cross y-axis equals z-axis")
	{
		ufo::Vec3d x(1.0, 0.0, 0.0);
		ufo::Vec3d y(0.0, 1.0, 0.0);
		auto       r = cross(x, y);
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(1.0));
	}

	SECTION("y-axis cross x-axis equals negative z-axis")
	{
		ufo::Vec3d x(1.0, 0.0, 0.0);
		ufo::Vec3d y(0.0, 1.0, 0.0);
		auto       r = cross(y, x);
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(-1.0));
	}

	SECTION("Self-cross product is zero")
	{
		ufo::Vec3d v(1.0, 2.0, 3.0);
		auto       r = cross(v, v);
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Cross product of arbitrary vectors")
	{
		// (1,0,0) x (0,0,1) = (0*1-0*0, 0*0-1*1, 1*0-0*0) = (0,-1,0)
		ufo::Vec3d v1(1.0, 0.0, 0.0);
		ufo::Vec3d v2(0.0, 0.0, 1.0);
		auto       r = cross(v1, v2);
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(-1.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[Vec3d] [length] Vector length")
{
	SECTION("Length of zero vector")
	{
		ufo::Vec3d v(0.0, 0.0, 0.0);
		REQUIRE(length(v) == Catch::Approx(0.0));
	}

	SECTION("Length of unit vector along x-axis")
	{
		ufo::Vec3d v(1.0, 0.0, 0.0);
		REQUIRE(length(v) == Catch::Approx(1.0));
	}

	SECTION("Length of unit vector along y-axis")
	{
		ufo::Vec3d v(0.0, 1.0, 0.0);
		REQUIRE(length(v) == Catch::Approx(1.0));
	}

	SECTION("Length of unit vector along z-axis")
	{
		ufo::Vec3d v(0.0, 0.0, 1.0);
		REQUIRE(length(v) == Catch::Approx(1.0));
	}

	SECTION("Length of 3-4-0 vector")
	{
		ufo::Vec3d v(3.0, 4.0, 0.0);
		REQUIRE(length(v) == Catch::Approx(5.0));
	}

	SECTION("Length of vector with negative components")
	{
		ufo::Vec3d v(-3.0, -4.0, 0.0);
		REQUIRE(length(v) == Catch::Approx(5.0));
	}

	SECTION("Length of equal-component vector")
	{
		ufo::Vec3d v(1.0, 1.0, 1.0);
		REQUIRE(length(v) == Catch::Approx(std::sqrt(3.0)));
	}
}

TEST_CASE("[Vec3d] [normalize]")
{
	SECTION("Normalize zero vector yields NaN components")
	{
		ufo::Vec3d v(0.0, 0.0, 0.0);
		auto       r = normalize(v);
		REQUIRE(!std::isfinite(r[0]));
		REQUIRE(!std::isfinite(r[1]));
		REQUIRE(!std::isfinite(r[2]));
	}

	SECTION("Normalize unit vector along x-axis")
	{
		ufo::Vec3d v(1.0, 0.0, 0.0);
		auto       r = normalize(v);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Normalize arbitrary vector has unit length")
	{
		ufo::Vec3d v(3.0, 4.0, 0.0);
		auto       r = normalize(v);
		REQUIRE(r[0] == Catch::Approx(0.6));
		REQUIRE(r[1] == Catch::Approx(0.8));
		REQUIRE(r[2] == Catch::Approx(0.0));
		REQUIRE(length(r) == Catch::Approx(1.0));
	}

	SECTION("Normalize vector with negative components")
	{
		ufo::Vec3d v(-3.0, 0.0, 4.0);
		auto       r = normalize(v);
		REQUIRE(r[0] == Catch::Approx(-0.6));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.8));
		REQUIRE(length(r) == Catch::Approx(1.0));
	}
}

TEST_CASE("[Vec3d] [distance]")
{
	SECTION("Distance between identical vectors")
	{
		ufo::Vec3d v1(1.0, 2.0, 3.0);
		ufo::Vec3d v2(1.0, 2.0, 3.0);
		REQUIRE(distance(v1, v2) == Catch::Approx(0.0));
	}

	SECTION("Distance between zero vectors")
	{
		ufo::Vec3d v1(0.0, 0.0, 0.0);
		ufo::Vec3d v2(0.0, 0.0, 0.0);
		REQUIRE(distance(v1, v2) == Catch::Approx(0.0));
	}

	SECTION("Distance between orthogonal unit vectors")
	{
		ufo::Vec3d v1(1.0, 0.0, 0.0);
		ufo::Vec3d v2(0.0, 1.0, 0.0);
		REQUIRE(distance(v1, v2) == Catch::Approx(std::sqrt(2.0)));
	}

	SECTION("Distance from origin along axis")
	{
		ufo::Vec3d v1(0.0, 0.0, 0.0);
		ufo::Vec3d v2(3.0, 4.0, 0.0);
		REQUIRE(distance(v1, v2) == Catch::Approx(5.0));
	}

	SECTION("Distance of arbitrary vectors")
	{
		ufo::Vec3d v1(1.0, 2.0, 2.0);
		ufo::Vec3d v2(4.0, 6.0, 2.0);
		REQUIRE(distance(v1, v2) == Catch::Approx(5.0));
	}
}

TEST_CASE("[Vec3d] [min]")
{
	SECTION("Minimum of zero vector")
	{
		ufo::Vec3d v(0.0, 0.0, 0.0);
		REQUIRE(min(v) == Catch::Approx(0.0));
	}

	SECTION("Minimum of positive vector")
	{
		ufo::Vec3d v(3.0, 1.5, 2.0);
		REQUIRE(min(v) == Catch::Approx(1.5));
	}

	SECTION("Minimum with negative components")
	{
		ufo::Vec3d v(-2.0, -4.0, -1.0);
		REQUIRE(min(v) == Catch::Approx(-4.0));
	}

	SECTION("Minimum of equal-component vector")
	{
		ufo::Vec3d v(2.0, 2.0, 2.0);
		REQUIRE(min(v) == Catch::Approx(2.0));
	}
}

TEST_CASE("[Vec3d] [max]")
{
	SECTION("Maximum of zero vector")
	{
		ufo::Vec3d v(0.0, 0.0, 0.0);
		REQUIRE(max(v) == Catch::Approx(0.0));
	}

	SECTION("Maximum of positive vector")
	{
		ufo::Vec3d v(3.0, 1.5, 2.0);
		REQUIRE(max(v) == Catch::Approx(3.0));
	}

	SECTION("Maximum with negative components")
	{
		ufo::Vec3d v(-2.0, -4.0, -1.0);
		REQUIRE(max(v) == Catch::Approx(-1.0));
	}
}

TEST_CASE("[Vec3d] [minIndex]")
{
	SECTION("Min index of arbitrary vector")
	{
		ufo::Vec3d v(3.0, 1.0, 2.0);
		REQUIRE(minIndex(v) == 1);
	}

	SECTION("Min index with negative components")
	{
		ufo::Vec3d v(-1.0, -4.0, -2.0);
		REQUIRE(minIndex(v) == 1);
	}

	SECTION("Min index of equal-component vector")
	{
		ufo::Vec3d v(2.0, 2.0, 2.0);
		auto       i = minIndex(v);
		REQUIRE((i == 0 || i == 1 || i == 2));
	}
}

TEST_CASE("[Vec3d] [maxIndex]")
{
	SECTION("Max index of arbitrary vector")
	{
		ufo::Vec3d v(3.0, 5.0, 1.0);
		REQUIRE(maxIndex(v) == 1);
	}

	SECTION("Max index with negative components")
	{
		ufo::Vec3d v(-1.0, -4.0, -2.0);
		REQUIRE(maxIndex(v) == 0);
	}

	SECTION("Max index of equal-component vector")
	{
		ufo::Vec3d v(2.0, 2.0, 2.0);
		auto       i = maxIndex(v);
		REQUIRE((i == 0 || i == 1 || i == 2));
	}
}

TEST_CASE("[Vec3d] [ceil]")
{
	SECTION("Ceiling of zero vector")
	{
		ufo::Vec3d r = ceil(ufo::Vec3d(0.0, 0.0, 0.0));
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Ceiling of positive fractional components")
	{
		ufo::Vec3d r = ceil(ufo::Vec3d(1.2, 2.7, 0.1));
		REQUIRE(r[0] == Catch::Approx(2.0));
		REQUIRE(r[1] == Catch::Approx(3.0));
		REQUIRE(r[2] == Catch::Approx(1.0));
	}

	SECTION("Ceiling of negative fractional components")
	{
		ufo::Vec3d r = ceil(ufo::Vec3d(-1.5, -2.7, -0.1));
		REQUIRE(r[0] == Catch::Approx(-1.0));
		REQUIRE(r[1] == Catch::Approx(-2.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Ceiling of mixed components")
	{
		ufo::Vec3d r = ceil(ufo::Vec3d(1.2, -3.8, 0.0));
		REQUIRE(r[0] == Catch::Approx(2.0));
		REQUIRE(r[1] == Catch::Approx(-3.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[Vec3d] [floor]")
{
	SECTION("Floor of zero vector")
	{
		ufo::Vec3d r = floor(ufo::Vec3d(0.0, 0.0, 0.0));
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Floor of positive fractional components")
	{
		ufo::Vec3d r = floor(ufo::Vec3d(1.9, 2.1, 3.5));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("Floor of negative fractional components")
	{
		ufo::Vec3d r = floor(ufo::Vec3d(-1.5, -2.7, -0.1));
		REQUIRE(r[0] == Catch::Approx(-2.0));
		REQUIRE(r[1] == Catch::Approx(-3.0));
		REQUIRE(r[2] == Catch::Approx(-1.0));
	}

	SECTION("Floor of mixed components")
	{
		ufo::Vec3d r = floor(ufo::Vec3d(1.9, -3.2, 0.0));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(-4.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[Vec3d] [trunc]")
{
	SECTION("Truncate of positive fractional components")
	{
		ufo::Vec3d r = trunc(ufo::Vec3d(1.9, 2.7, 3.1));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("Truncate of negative fractional components")
	{
		ufo::Vec3d r = trunc(ufo::Vec3d(-1.9, -2.7, -0.1));
		REQUIRE(r[0] == Catch::Approx(-1.0));
		REQUIRE(r[1] == Catch::Approx(-2.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Truncate of mixed components")
	{
		ufo::Vec3d r = trunc(ufo::Vec3d(1.9, -3.8, 0.0));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(-3.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[Vec3d] [round]")
{
	SECTION("Round of positive fractional components")
	{
		ufo::Vec3d r = round(ufo::Vec3d(1.5, 2.4, 3.6));
		REQUIRE(r[0] == Catch::Approx(2.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(4.0));
	}

	SECTION("Round of negative fractional components")
	{
		ufo::Vec3d r = round(ufo::Vec3d(-1.5, -2.4, -3.6));
		REQUIRE(r[0] == Catch::Approx(-2.0));
		REQUIRE(r[1] == Catch::Approx(-2.0));
		REQUIRE(r[2] == Catch::Approx(-4.0));
	}

	SECTION("Round of mixed components")
	{
		ufo::Vec3d r = round(ufo::Vec3d(1.2, -3.8, 0.5));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(-4.0));
		REQUIRE(r[2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Vec3d] [abs]")
{
	SECTION("Absolute value of zero vector")
	{
		ufo::Vec3d r = abs(ufo::Vec3d(0.0, 0.0, 0.0));
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Absolute value of positive components")
	{
		ufo::Vec3d r = abs(ufo::Vec3d(1.5, 2.0, 3.7));
		REQUIRE(r[0] == Catch::Approx(1.5));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.7));
	}

	SECTION("Absolute value of negative components")
	{
		ufo::Vec3d r = abs(ufo::Vec3d(-1.5, -2.0, -3.7));
		REQUIRE(r[0] == Catch::Approx(1.5));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.7));
	}

	SECTION("Absolute value of mixed components")
	{
		ufo::Vec3d r = abs(ufo::Vec3d(1.2, -3.8, 0.0));
		REQUIRE(r[0] == Catch::Approx(1.2));
		REQUIRE(r[1] == Catch::Approx(3.8));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[Vec3d] [clamp]")
{
	ufo::Vec3d lo(-1.0, -1.0, -1.0);
	ufo::Vec3d hi(1.0, 1.0, 1.0);

	SECTION("Clamping of zero vector stays zero")
	{
		ufo::Vec3d r = clamp(ufo::Vec3d(0.0, 0.0, 0.0), lo, hi);
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Clamping of vector within range is unchanged")
	{
		ufo::Vec3d r = clamp(ufo::Vec3d(0.5, -0.5, 0.0), lo, hi);
		REQUIRE(r[0] == Catch::Approx(0.5));
		REQUIRE(r[1] == Catch::Approx(-0.5));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Clamping of components above maximum")
	{
		ufo::Vec3d r = clamp(ufo::Vec3d(1.5, 2.5, 3.0), lo, hi);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(1.0));
	}

	SECTION("Clamping of components below minimum")
	{
		ufo::Vec3d r = clamp(ufo::Vec3d(-1.5, -2.5, -3.0), lo, hi);
		REQUIRE(r[0] == Catch::Approx(-1.0));
		REQUIRE(r[1] == Catch::Approx(-1.0));
		REQUIRE(r[2] == Catch::Approx(-1.0));
	}
}

TEST_CASE("[Vec3d] [lerp]")
{
	SECTION("Linear interpolation between zero vector and one vector")
	{
		ufo::Vec3d v0(0.0, 0.0, 0.0);
		ufo::Vec3d v1(1.0, 1.0, 1.0);
		double     t = 0.5;
		ufo::Vec3d r = ufo::lerp(v0, v1, t);
		REQUIRE(r[0] == Catch::Approx(0.5));
		REQUIRE(r[1] == Catch::Approx(0.5));
		REQUIRE(r[2] == Catch::Approx(0.5));
	}

	SECTION("Linear interpolation with t=0 returns first vector")
	{
		ufo::Vec3d v0(1.0, 2.0, 3.0);
		ufo::Vec3d v1(4.0, 5.0, 6.0);
		double     t = 0.0;
		ufo::Vec3d r = ufo::lerp(v0, v1, t);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("Linear interpolation with t=1 returns second vector")
	{
		ufo::Vec3d v0(1.0, 2.0, 3.0);
		ufo::Vec3d v1(4.0, 5.0, 6.0);
		double     t = 1.0;
		ufo::Vec3d r = ufo::lerp(v0, v1, t);
		REQUIRE(r[0] == Catch::Approx(4.0));
		REQUIRE(r[1] == Catch::Approx(5.0));
		REQUIRE(r[2] == Catch::Approx(6.0));
	}

	SECTION("Linear interpolation with t=0.25")
	{
		ufo::Vec3d v0(1.0, 2.0, 3.0);
		ufo::Vec3d v1(4.0, 5.0, 6.0);
		double     t = 0.25;
		ufo::Vec3d r = ufo::lerp(v0, v1, t);
		REQUIRE(r[0] == Catch::Approx(1.75));
		REQUIRE(r[1] == Catch::Approx(2.75));
		REQUIRE(r[2] == Catch::Approx(3.75));
	}
}

TEST_CASE("[vec] [x/y/z/w] Named accessors")
{
	SECTION("Vec3d x/y/z match subscript")
	{
		ufo::Vec3d v(1.0, 2.0, 3.0);
		REQUIRE(v.x() == v[0]);
		REQUIRE(v.y() == v[1]);
		REQUIRE(v.z() == v[2]);
	}

	SECTION("Vec4d w matches subscript")
	{
		ufo::Vec4d v(1.0, 2.0, 3.0, 4.0);
		REQUIRE(v.w() == v[3]);
	}

	SECTION("Named accessors are mutable")
	{
		ufo::Vec3d v(0.0, 0.0, 0.0);
		v.x() = 1.0;
		v.y() = 2.0;
		v.z() = 3.0;
		REQUIRE(v[0] == 1.0);
		REQUIRE(v[1] == 2.0);
		REQUIRE(v[2] == 3.0);
	}
}

TEST_CASE("[vec] [distanceSquared] Squared distance")
{
	SECTION("Same point")
	{
		REQUIRE(ufo::distanceSquared(ufo::Vec3d(1, 2, 3), ufo::Vec3d(1, 2, 3)) ==
		        Catch::Approx(0.0));
	}

	SECTION("3-4-0 triangle")
	{
		REQUIRE(ufo::distanceSquared(ufo::Vec3d(0, 0, 0), ufo::Vec3d(3, 4, 0)) ==
		        Catch::Approx(25.0));
	}

	SECTION("Equals distance squared")
	{
		ufo::Vec3d a(1.0, 2.0, 2.0), b(4.0, 6.0, 2.0);
		double     d = ufo::distance(a, b);
		REQUIRE(ufo::distanceSquared(a, b) == Catch::Approx(d * d));
	}
}

TEST_CASE("[vec] [isNormalized] Unit-length check")
{
	SECTION("Unit x-axis") { REQUIRE(ufo::isNormalized(ufo::Vec3d(1.0, 0.0, 0.0))); }

	SECTION("Non-unit vector") { REQUIRE(!ufo::isNormalized(ufo::Vec3d(2.0, 0.0, 0.0))); }

	SECTION("Non-unit diagonal") { REQUIRE(!ufo::isNormalized(ufo::Vec3d(1.0, 1.0, 1.0))); }

	SECTION("Normalized vector")
	{
		REQUIRE(ufo::isNormalized(ufo::normalize(ufo::Vec3d(3.0, 4.0, 0.0))));
	}
}

TEST_CASE("[vec] [reflect] Reflection about a surface normal")
{
	SECTION("45-degree incidence")
	{
		ufo::Vec3d v(1.0, -1.0, 0.0);
		ufo::Vec3d n(0.0, 1.0, 0.0);
		auto       r = ufo::reflect(v, n);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}

	SECTION("Perpendicular incidence reverses normal component")
	{
		ufo::Vec3d v(0.0, -1.0, 0.0);
		ufo::Vec3d n(0.0, 1.0, 0.0);
		auto       r = ufo::reflect(v, n);
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[vec] [sum] Element sum")
{
	SECTION("Zero vector")
	{
		REQUIRE(ufo::sum(ufo::Vec3d(0.0, 0.0, 0.0)) == Catch::Approx(0.0));
	}

	SECTION("Positive components")
	{
		REQUIRE(ufo::sum(ufo::Vec3d(1.0, 2.0, 3.0)) == Catch::Approx(6.0));
	}

	SECTION("Mixed signs")
	{
		REQUIRE(ufo::sum(ufo::Vec3d(1.0, -1.0, 2.0)) == Catch::Approx(2.0));
	}
}

TEST_CASE("[vec] [product] Element product")
{
	SECTION("Contains zero")
	{
		REQUIRE(ufo::product(ufo::Vec3d(0.0, 2.0, 3.0)) == Catch::Approx(0.0));
	}

	SECTION("All ones")
	{
		REQUIRE(ufo::product(ufo::Vec3d(1.0, 1.0, 1.0)) == Catch::Approx(1.0));
	}

	SECTION("Positive components")
	{
		REQUIRE(ufo::product(ufo::Vec3d(2.0, 3.0, 4.0)) == Catch::Approx(24.0));
	}
}

TEST_CASE("[vec] [min(v,v)] Component-wise minimum")
{
	ufo::Vec3d v1(1.0, 5.0, 3.0);
	ufo::Vec3d v2(4.0, 2.0, 3.0);
	auto       r = ufo::min(v1, v2);
	REQUIRE(r[0] == Catch::Approx(1.0));
	REQUIRE(r[1] == Catch::Approx(2.0));
	REQUIRE(r[2] == Catch::Approx(3.0));
}

TEST_CASE("[vec] [max(v,v)] Component-wise maximum")
{
	ufo::Vec3d v1(1.0, 5.0, 3.0);
	ufo::Vec3d v2(4.0, 2.0, 3.0);
	auto       r = ufo::max(v1, v2);
	REQUIRE(r[0] == Catch::Approx(4.0));
	REQUIRE(r[1] == Catch::Approx(5.0));
	REQUIRE(r[2] == Catch::Approx(3.0));
}

TEST_CASE("[vec] [equal] Component-wise equality")
{
	ufo::Vec3d v1(1.0, 2.0, 3.0), v2(1.0, 3.0, 3.0);
	auto       r = ufo::equal(v1, v2);
	REQUIRE(r[0] == true);
	REQUIRE(r[1] == false);
	REQUIRE(r[2] == true);
}

TEST_CASE("[vec] [notEqual] Component-wise inequality")
{
	ufo::Vec3d v1(1.0, 2.0, 3.0), v2(1.0, 3.0, 3.0);
	auto       r = ufo::notEqual(v1, v2);
	REQUIRE(r[0] == false);
	REQUIRE(r[1] == true);
	REQUIRE(r[2] == false);
}

TEST_CASE("[vec] [less/lessEqual/greater/greaterEqual] Component-wise ordering")
{
	ufo::Vec3d v1(1.0, 3.0, 2.0), v2(2.0, 2.0, 2.0);

	SECTION("less")
	{
		auto r = ufo::less(v1, v2);
		REQUIRE(r[0] == true);
		REQUIRE(r[1] == false);
		REQUIRE(r[2] == false);
	}

	SECTION("lessEqual")
	{
		auto r = ufo::lessEqual(v1, v2);
		REQUIRE(r[0] == true);
		REQUIRE(r[1] == false);
		REQUIRE(r[2] == true);
	}

	SECTION("greater")
	{
		auto r = ufo::greater(v1, v2);
		REQUIRE(r[0] == false);
		REQUIRE(r[1] == true);
		REQUIRE(r[2] == false);
	}

	SECTION("greaterEqual")
	{
		auto r = ufo::greaterEqual(v1, v2);
		REQUIRE(r[0] == false);
		REQUIRE(r[1] == true);
		REQUIRE(r[2] == true);
	}
}

TEST_CASE("[vec] [all/any/some/none] Boolean reductions")
{
	SECTION("all: all true") { REQUIRE(ufo::all(ufo::Vec3b(true, true, true))); }

	SECTION("all: any false") { REQUIRE(!ufo::all(ufo::Vec3b(true, false, true))); }

	SECTION("any: at least one true") { REQUIRE(ufo::any(ufo::Vec3b(false, true, false))); }

	SECTION("any: all false") { REQUIRE(!ufo::any(ufo::Vec3b(false, false, false))); }

	SECTION("some: mixed") { REQUIRE(ufo::some(ufo::Vec3b(true, false, true))); }

	SECTION("some: all true") { REQUIRE(!ufo::some(ufo::Vec3b(true, true, true))); }

	SECTION("some: all false") { REQUIRE(!ufo::some(ufo::Vec3b(false, false, false))); }

	SECTION("none: all false") { REQUIRE(ufo::none(ufo::Vec3b(false, false, false))); }

	SECTION("none: any true") { REQUIRE(!ufo::none(ufo::Vec3b(false, true, false))); }
}

TEST_CASE("[vec] [isnan/isfinite/isnormal] Floating-point classification")
{
	SECTION("isnan: NaN component")
	{
		REQUIRE(ufo::isnan(ufo::Vec3d(1.0, std::numeric_limits<double>::quiet_NaN(), 3.0)));
	}

	SECTION("isnan: finite vector") { REQUIRE(!ufo::isnan(ufo::Vec3d(1.0, 2.0, 3.0))); }

	SECTION("isfinite: finite vector")
	{
		REQUIRE(ufo::isfinite(ufo::Vec3d(1.0, 2.0, 3.0)));
	}

	SECTION("isfinite: infinite component")
	{
		REQUIRE(
		    !ufo::isfinite(ufo::Vec3d(1.0, std::numeric_limits<double>::infinity(), 3.0)));
	}

	SECTION("isnormal: normal vector")
	{
		REQUIRE(ufo::isnormal(ufo::Vec3d(1.0, 2.0, 3.0)));
	}

	SECTION("isnormal: subnormal component")
	{
		REQUIRE(
		    !ufo::isnormal(ufo::Vec3d(1.0, std::numeric_limits<double>::denorm_min(), 3.0)));
	}
}

TEST_CASE("[vec] [mix] Blend two vectors")
{
	SECTION("Bool mask selects y when true, x when false")
	{
		ufo::Vec3d x(1.0, 2.0, 3.0), y(4.0, 5.0, 6.0);
		auto       r = ufo::mix(x, y, ufo::Vec3b(true, false, true));
		REQUIRE(r[0] == Catch::Approx(4.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(6.0));
	}

	SECTION("Scalar weight 0 returns x")
	{
		ufo::Vec3d x(1.0, 2.0, 3.0), y(4.0, 5.0, 6.0);
		auto       r = ufo::mix(x, y, ufo::Vec3d(0.0, 0.0, 0.0));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("Scalar weight 1 returns y")
	{
		ufo::Vec3d x(1.0, 2.0, 3.0), y(4.0, 5.0, 6.0);
		auto       r = ufo::mix(x, y, ufo::Vec3d(1.0, 1.0, 1.0));
		REQUIRE(r[0] == Catch::Approx(4.0));
		REQUIRE(r[1] == Catch::Approx(5.0));
		REQUIRE(r[2] == Catch::Approx(6.0));
	}

	SECTION("Scalar weight 0.5 gives midpoint")
	{
		ufo::Vec3d x(0.0, 0.0, 0.0), y(2.0, 4.0, 6.0);
		auto       r = ufo::mix(x, y, ufo::Vec3d(0.5, 0.5, 0.5));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}
}

TEST_CASE("[vec] [sign] Component-wise sign")
{
	SECTION("Positive components")
	{
		auto r = ufo::sign(ufo::Vec3d(3.0, 0.5, 100.0));
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(1.0));
	}

	SECTION("Negative components")
	{
		auto r = ufo::sign(ufo::Vec3d(-3.0, -0.5, -100.0));
		REQUIRE(r[0] == Catch::Approx(-1.0));
		REQUIRE(r[1] == Catch::Approx(-1.0));
		REQUIRE(r[2] == Catch::Approx(-1.0));
	}

	SECTION("Zero components")
	{
		auto r = ufo::sign(ufo::Vec3d(0.0, 0.0, 0.0));
		REQUIRE(r[0] == Catch::Approx(0.0));
		REQUIRE(r[1] == Catch::Approx(0.0));
		REQUIRE(r[2] == Catch::Approx(0.0));
	}
}

TEST_CASE("[vec] [operator%] Integer modulo")
{
	ufo::Vec3i v(10, 11, 12);

	SECTION("vec % vec")
	{
		auto r = v % ufo::Vec3i(3, 4, 5);
		REQUIRE(r[0] == 1);
		REQUIRE(r[1] == 3);
		REQUIRE(r[2] == 2);
	}

	SECTION("vec % scalar")
	{
		auto r = v % 4;
		REQUIRE(r[0] == 2);
		REQUIRE(r[1] == 3);
		REQUIRE(r[2] == 0);
	}
}

TEST_CASE("[vec] [operator&] Bitwise AND")
{
	SECTION("vec & vec")
	{
		auto r = ufo::Vec3i(6, 5, 3) & ufo::Vec3i(3, 3, 3);
		REQUIRE(r[0] == (6 & 3));
		REQUIRE(r[1] == (5 & 3));
		REQUIRE(r[2] == (3 & 3));
	}

	SECTION("vec & scalar")
	{
		auto r = ufo::Vec3i(6, 5, 3) & 5;
		REQUIRE(r[0] == (6 & 5));
		REQUIRE(r[1] == (5 & 5));
		REQUIRE(r[2] == (3 & 5));
	}
}

TEST_CASE("[vec] [operator|] Bitwise OR")
{
	SECTION("vec | vec")
	{
		auto r = ufo::Vec3i(4, 2, 1) | ufo::Vec3i(2, 1, 4);
		REQUIRE(r[0] == (4 | 2));
		REQUIRE(r[1] == (2 | 1));
		REQUIRE(r[2] == (1 | 4));
	}

	SECTION("vec | scalar")
	{
		auto r = ufo::Vec3i(4, 2, 1) | 3;
		REQUIRE(r[0] == (4 | 3));
		REQUIRE(r[1] == (2 | 3));
		REQUIRE(r[2] == (1 | 3));
	}
}

TEST_CASE("[vec] [operator^] Bitwise XOR")
{
	SECTION("vec ^ vec")
	{
		auto r = ufo::Vec3i(6, 5, 3) ^ ufo::Vec3i(3, 3, 3);
		REQUIRE(r[0] == (6 ^ 3));
		REQUIRE(r[1] == (5 ^ 3));
		REQUIRE(r[2] == (3 ^ 3));
	}

	SECTION("vec ^ scalar")
	{
		auto r = ufo::Vec3i(6, 5, 3) ^ 3;
		REQUIRE(r[0] == (6 ^ 3));
		REQUIRE(r[1] == (5 ^ 3));
		REQUIRE(r[2] == (3 ^ 3));
	}
}

TEST_CASE("[vec] [operator<<] Left shift")
{
	ufo::Vec3i v(1, 2, 4);

	SECTION("vec << vec")
	{
		auto r = v << ufo::Vec3i(1, 2, 3);
		REQUIRE(r[0] == 2);
		REQUIRE(r[1] == 8);
		REQUIRE(r[2] == 32);
	}

	SECTION("vec << scalar")
	{
		auto r = v << 2;
		REQUIRE(r[0] == 4);
		REQUIRE(r[1] == 8);
		REQUIRE(r[2] == 16);
	}
}

TEST_CASE("[vec] [operator>>] Right shift")
{
	ufo::Vec3i v(16, 8, 4);

	SECTION("vec >> vec")
	{
		auto r = v >> ufo::Vec3i(1, 2, 1);
		REQUIRE(r[0] == 8);
		REQUIRE(r[1] == 2);
		REQUIRE(r[2] == 2);
	}

	SECTION("vec >> scalar")
	{
		auto r = v >> 1;
		REQUIRE(r[0] == 8);
		REQUIRE(r[1] == 4);
		REQUIRE(r[2] == 2);
	}
}

TEST_CASE("[vec] [operator~] Bitwise NOT")
{
	ufo::Vec3i v(0, 1, -1);
	auto       r = ~v;
	REQUIRE(r[0] == ~0);
	REQUIRE(r[1] == ~1);
	REQUIRE(r[2] == ~(-1));
}