// UFO
#include <ufo/math/quat.hpp>

// STL
#include <numbers>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[Quatd] [constructor] Default constructor is identity")
{
	ufo::Quatd q;
	REQUIRE(q.w == 1.0);
	REQUIRE(q.x == 0.0);
	REQUIRE(q.y == 0.0);
	REQUIRE(q.z == 0.0);
}

TEST_CASE("[Quatd] [constructor] Construct from components")
{
	ufo::Quatd q(0.5, 0.5, 0.5, 0.5);
	REQUIRE(q.w == 0.5);
	REQUIRE(q.x == 0.5);
	REQUIRE(q.y == 0.5);
	REQUIRE(q.z == 0.5);
}

TEST_CASE("[Quatd] [constructor] Construct from rotation matrix")
{
	// 90-degree rotation around z: rotation matrix
	auto m = ufo::Mat3d(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	SECTION("Quaternion from identity matrix is identity quat")
	{
		ufo::Quatd q(ufo::Mat3d::identity());
		REQUIRE(q.w == Catch::Approx(1.0));
		REQUIRE(q.x == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(q.y == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(q.z == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("Round-trip: Mat -> Quat -> Mat is unchanged")
	{
		ufo::Quatd q(m);
		auto       recovered = ufo::Mat3d(q);
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				REQUIRE(recovered[r][c] == Catch::Approx(m[r][c]).margin(1e-10));
	}
}

TEST_CASE("[Quatd] [operator==] Equality operator")
{
	ufo::Quatd q1(1.0, 0.0, 0.0, 0.0);
	ufo::Quatd q2(1.0, 0.0, 0.0, 0.0);
	REQUIRE(q1 == q2);
}

TEST_CASE("[Quatd] [operator!=] Inequality operator")
{
	ufo::Quatd q1(1.0, 0.0, 0.0, 0.0);
	ufo::Quatd q2(0.0, 1.0, 0.0, 0.0);
	REQUIRE(q1 != q2);
}

TEST_CASE("[Quatd] [operator+] Addition operator")
{
	SECTION("Unary plus returns same quaternion")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		REQUIRE(+q == q);
	}

	SECTION("Adding two quaternions component-wise")
	{
		ufo::Quatd q1(1.0, 2.0, 3.0, 4.0);
		ufo::Quatd q2(0.5, 0.5, 0.5, 0.5);
		auto       r = q1 + q2;
		REQUIRE(r.w == Catch::Approx(1.5));
		REQUIRE(r.x == Catch::Approx(2.5));
		REQUIRE(r.y == Catch::Approx(3.5));
		REQUIRE(r.z == Catch::Approx(4.5));
	}
}

TEST_CASE("[Quatd] [operator-] Subtraction operator")
{
	SECTION("Unary negation negates all components")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		auto       r = -q;
		REQUIRE(r.w == -1.0);
		REQUIRE(r.x == -2.0);
		REQUIRE(r.y == -3.0);
		REQUIRE(r.z == -4.0);
	}

	SECTION("Subtracting two quaternions component-wise")
	{
		ufo::Quatd q1(1.0, 2.0, 3.0, 4.0);
		ufo::Quatd q2(0.5, 0.5, 0.5, 0.5);
		auto       r = q1 - q2;
		REQUIRE(r.w == Catch::Approx(0.5));
		REQUIRE(r.x == Catch::Approx(1.5));
		REQUIRE(r.y == Catch::Approx(2.5));
		REQUIRE(r.z == Catch::Approx(3.5));
	}
}

TEST_CASE("[Quatd] [operator*] Multiplication operator")
{
	SECTION("identity * q = q")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		ufo::Quatd q(0.0, 1.0, 0.0, 0.0);
		auto       r = id * q;
		REQUIRE(r.w == Catch::Approx(q.w));
		REQUIRE(r.x == Catch::Approx(q.x));
		REQUIRE(r.y == Catch::Approx(q.y));
		REQUIRE(r.z == Catch::Approx(q.z));
	}

	SECTION("q * identity = q")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		ufo::Quatd q(0.0, 1.0, 0.0, 0.0);
		auto       r = q * id;
		REQUIRE(r.w == Catch::Approx(q.w));
		REQUIRE(r.x == Catch::Approx(q.x));
		REQUIRE(r.y == Catch::Approx(q.y));
		REQUIRE(r.z == Catch::Approx(q.z));
	}

	SECTION("Pure-i * pure-i = -identity (i^2 = -1)")
	{
		ufo::Quatd i(0.0, 1.0, 0.0, 0.0);
		auto       r = i * i;
		REQUIRE(r.w == Catch::Approx(-1.0));
		REQUIRE(r.x == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.y == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.z == Catch::Approx(0.0).margin(1e-15));
	}

	SECTION("pure-i * pure-j = pure-k (i*j = k)")
	{
		ufo::Quatd i(0.0, 1.0, 0.0, 0.0);
		ufo::Quatd j(0.0, 0.0, 1.0, 0.0);
		auto       r = i * j;
		REQUIRE(r.w == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.x == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.y == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.z == Catch::Approx(1.0));
	}

	SECTION("Multiplication is not commutative (i*j != j*i)")
	{
		ufo::Quatd i(0.0, 1.0, 0.0, 0.0);
		ufo::Quatd j(0.0, 0.0, 1.0, 0.0);
		REQUIRE(i * j != j * i);
	}

	SECTION("quat * scalar scales all components")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		auto       r = q * 2.0;
		REQUIRE(r.w == Catch::Approx(2.0));
		REQUIRE(r.x == Catch::Approx(4.0));
		REQUIRE(r.y == Catch::Approx(6.0));
		REQUIRE(r.z == Catch::Approx(8.0));
	}

	SECTION("scalar * quat scales all components")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		auto       r = 2.0 * q;
		REQUIRE(r.w == Catch::Approx(2.0));
		REQUIRE(r.x == Catch::Approx(4.0));
		REQUIRE(r.y == Catch::Approx(6.0));
		REQUIRE(r.z == Catch::Approx(8.0));
	}

	SECTION("90-degree z-rotation applied to x-axis gives y-axis")
	{
		ufo::Quatd q = angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		auto       r = q * ufo::Vec3d(1.0, 0.0, 0.0);
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("180-degree z-rotation applied to x-axis gives negative x-axis")
	{
		ufo::Quatd q = angleAxis(std::numbers::pi, ufo::Vec3d(0.0, 0.0, 1.0));
		auto       r = q * ufo::Vec3d(1.0, 0.0, 0.0);
		REQUIRE(r[0] == Catch::Approx(-1.0));
		REQUIRE(r[1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
	}
}

TEST_CASE("[Quatd] [operator/] Division by scalar")
{
	ufo::Quatd q(2.0, 4.0, 6.0, 8.0);
	auto       r = q / 2.0;
	REQUIRE(r.w == Catch::Approx(1.0));
	REQUIRE(r.x == Catch::Approx(2.0));
	REQUIRE(r.y == Catch::Approx(3.0));
	REQUIRE(r.z == Catch::Approx(4.0));
}

TEST_CASE("[Quatd] [operator+=] Compound addition")
{
	ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
	q += ufo::Quatd(0.5, 0.5, 0.5, 0.5);
	REQUIRE(q.w == Catch::Approx(1.5));
	REQUIRE(q.x == Catch::Approx(2.5));
	REQUIRE(q.y == Catch::Approx(3.5));
	REQUIRE(q.z == Catch::Approx(4.5));
}

TEST_CASE("[Quatd] [operator-=] Compound subtraction")
{
	ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
	q -= ufo::Quatd(0.5, 0.5, 0.5, 0.5);
	REQUIRE(q.w == Catch::Approx(0.5));
	REQUIRE(q.x == Catch::Approx(1.5));
	REQUIRE(q.y == Catch::Approx(2.5));
	REQUIRE(q.z == Catch::Approx(3.5));
}

TEST_CASE("[Quatd] [operator*=] Compound multiplication")
{
	SECTION("Compound multiplication by quaternion")
	{
		ufo::Quatd q(1.0, 0.0, 0.0, 0.0);
		ufo::Quatd i(0.0, 1.0, 0.0, 0.0);
		q *= i;
		REQUIRE(q.w == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(q.x == Catch::Approx(1.0));
	}

	SECTION("Compound multiplication by scalar")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		q *= 2.0;
		REQUIRE(q.w == Catch::Approx(2.0));
		REQUIRE(q.x == Catch::Approx(4.0));
		REQUIRE(q.y == Catch::Approx(6.0));
		REQUIRE(q.z == Catch::Approx(8.0));
	}
}

TEST_CASE("[Quatd] [operator/=] Compound division by scalar")
{
	ufo::Quatd q(2.0, 4.0, 6.0, 8.0);
	q /= 2.0;
	REQUIRE(q.w == Catch::Approx(1.0));
	REQUIRE(q.x == Catch::Approx(2.0));
	REQUIRE(q.y == Catch::Approx(3.0));
	REQUIRE(q.z == Catch::Approx(4.0));
}

TEST_CASE("[Quatd] [operator[]] Subscript operator")
{
	ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
	REQUIRE(q[0] == 1.0);  // w
	REQUIRE(q[1] == 2.0);  // x
	REQUIRE(q[2] == 3.0);  // y
	REQUIRE(q[3] == 4.0);  // z
}

TEST_CASE("[Quatd] [dot] Dot product")
{
	SECTION("Dot product of identity quaternions")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		REQUIRE(dot(id, id) == Catch::Approx(1.0));
	}

	SECTION("Dot product of orthogonal quaternions")
	{
		ufo::Quatd i(0.0, 1.0, 0.0, 0.0);
		ufo::Quatd j(0.0, 0.0, 1.0, 0.0);
		REQUIRE(dot(i, j) == Catch::Approx(0.0));
	}

	SECTION("Dot product of arbitrary quaternions")
	{
		ufo::Quatd q1(1.0, 2.0, 3.0, 4.0);
		ufo::Quatd q2(1.0, 1.0, 1.0, 1.0);
		REQUIRE(dot(q1, q2) == Catch::Approx(10.0));
	}
}

TEST_CASE("[Quatd] [norm] Quaternion norm")
{
	SECTION("Norm of identity quaternion is 1")
	{
		ufo::Quatd q(1.0, 0.0, 0.0, 0.0);
		REQUIRE(norm(q) == Catch::Approx(1.0));
	}

	SECTION("Norm of zero quaternion is 0")
	{
		ufo::Quatd q(0.0, 0.0, 0.0, 0.0);
		REQUIRE(norm(q) == Catch::Approx(0.0));
	}

	SECTION("Norm of (1,1,1,1) is 2")
	{
		ufo::Quatd q(1.0, 1.0, 1.0, 1.0);
		REQUIRE(norm(q) == Catch::Approx(2.0));
	}
}

TEST_CASE("[Quatd] [normalize] Normalization")
{
	SECTION("Normalize identity quaternion returns identity")
	{
		ufo::Quatd q(1.0, 0.0, 0.0, 0.0);
		auto       r = normalize(q);
		REQUIRE(norm(r) == Catch::Approx(1.0));
		REQUIRE(r == q);
	}

	SECTION("Normalize zero quaternion returns identity (safe fallback)")
	{
		ufo::Quatd q(0.0, 0.0, 0.0, 0.0);
		auto       r = normalize(q);
		REQUIRE(r.w == Catch::Approx(1.0));
		REQUIRE(r.x == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.y == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(r.z == Catch::Approx(0.0).margin(1e-15));
	}

	SECTION("Normalize arbitrary quaternion has unit norm")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		auto       r = normalize(q);
		REQUIRE(norm(r) == Catch::Approx(1.0));
		double n = norm(q);
		REQUIRE(r.w == Catch::Approx(q.w / n));
		REQUIRE(r.x == Catch::Approx(q.x / n));
		REQUIRE(r.y == Catch::Approx(q.y / n));
		REQUIRE(r.z == Catch::Approx(q.z / n));
	}

	SECTION("Normalization is idempotent")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		auto       n1 = normalize(q);
		auto       n2 = normalize(n1);
		REQUIRE(n1.w == Catch::Approx(n2.w));
		REQUIRE(n1.x == Catch::Approx(n2.x));
		REQUIRE(n1.y == Catch::Approx(n2.y));
		REQUIRE(n1.z == Catch::Approx(n2.z));
	}
}

TEST_CASE("[Quatd] [conjugate] Conjugate")
{
	SECTION("Conjugate negates x, y, z and keeps w")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		auto       c = conjugate(q);
		REQUIRE(c.w == Catch::Approx(1.0));
		REQUIRE(c.x == Catch::Approx(-2.0));
		REQUIRE(c.y == Catch::Approx(-3.0));
		REQUIRE(c.z == Catch::Approx(-4.0));
	}

	SECTION("Double conjugate returns original")
	{
		ufo::Quatd q(1.0, 2.0, 3.0, 4.0);
		REQUIRE(conjugate(conjugate(q)) == q);
	}

	SECTION("Conjugate of identity is identity")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		REQUIRE(conjugate(id) == id);
	}
}

TEST_CASE("[Quatd] [inverse] Quaternion inverse")
{
	SECTION("Inverse of identity is identity")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		auto       inv = inverse(id);
		REQUIRE(inv.w == Catch::Approx(1.0));
		REQUIRE(inv.x == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(inv.y == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(inv.z == Catch::Approx(0.0).margin(1e-15));
	}

	SECTION("q * inverse(q) = identity")
	{
		ufo::Quatd q = normalize(ufo::Quatd(1.0, 2.0, 3.0, 4.0));
		auto       r = q * inverse(q);
		REQUIRE(r.w == Catch::Approx(1.0));
		REQUIRE(r.x == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r.y == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r.z == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("For unit quaternion, inverse equals conjugate")
	{
		ufo::Quatd q   = normalize(ufo::Quatd(1.0, 2.0, 3.0, 4.0));
		auto       inv = inverse(q);
		auto       con = conjugate(q);
		REQUIRE(inv.w == Catch::Approx(con.w));
		REQUIRE(inv.x == Catch::Approx(con.x));
		REQUIRE(inv.y == Catch::Approx(con.y));
		REQUIRE(inv.z == Catch::Approx(con.z));
	}
}

TEST_CASE("[Quatd] [angleAxis] Angle-axis construction")
{
	SECTION("Zero angle gives identity quaternion")
	{
		auto q = angleAxis(0.0, ufo::Vec3d(0.0, 0.0, 1.0));
		REQUIRE(q.w == Catch::Approx(1.0));
		REQUIRE(q.x == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(q.y == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(q.z == Catch::Approx(0.0).margin(1e-15));
	}

	SECTION("90-degree z-rotation has expected components")
	{
		auto   q    = angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		double half = std::sqrt(2.0) / 2.0;
		REQUIRE(q.w == Catch::Approx(half));
		REQUIRE(q.x == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(q.y == Catch::Approx(0.0).margin(1e-15));
		REQUIRE(q.z == Catch::Approx(half));
	}

	SECTION("angleAxis produces unit quaternion")
	{
		auto q =
		    angleAxis(std::numbers::pi / 3.0, ufo::Vec3d(1.0, 1.0, 0.0) / std::sqrt(2.0));
		REQUIRE(norm(q) == Catch::Approx(1.0));
	}
}

TEST_CASE("[Quatd] [slerp] Spherical linear interpolation")
{
	ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
	ufo::Quatd q90z = angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));

	SECTION("slerp at t=0 returns first quaternion")
	{
		auto r = slerp(id, q90z, 0.0);
		REQUIRE(r.w == Catch::Approx(id.w));
		REQUIRE(r.x == Catch::Approx(id.x).margin(1e-10));
		REQUIRE(r.y == Catch::Approx(id.y).margin(1e-10));
		REQUIRE(r.z == Catch::Approx(id.z).margin(1e-10));
	}

	SECTION("slerp at t=1 returns second quaternion")
	{
		auto r = slerp(id, q90z, 1.0);
		REQUIRE(r.w == Catch::Approx(q90z.w));
		REQUIRE(r.x == Catch::Approx(q90z.x).margin(1e-10));
		REQUIRE(r.y == Catch::Approx(q90z.y).margin(1e-10));
		REQUIRE(r.z == Catch::Approx(q90z.z).margin(1e-10));
	}

	SECTION("slerp at t=0.5 is midpoint rotation (45 degrees around z)")
	{
		auto q45z = angleAxis(std::numbers::pi / 4.0, ufo::Vec3d(0.0, 0.0, 1.0));
		auto r    = slerp(id, q90z, 0.5);
		REQUIRE(r.w == Catch::Approx(q45z.w));
		REQUIRE(r.x == Catch::Approx(q45z.x).margin(1e-10));
		REQUIRE(r.y == Catch::Approx(q45z.y).margin(1e-10));
		REQUIRE(r.z == Catch::Approx(q45z.z));
	}

	SECTION("slerp result is always unit quaternion")
	{
		for (double t = 0.0; t <= 1.0; t += 0.1) {
			REQUIRE(norm(slerp(id, q90z, t)) == Catch::Approx(1.0).epsilon(1e-10));
		}
	}
}

TEST_CASE("[Quatd] [eulerAngles] Euler angle round-trip")
{
	SECTION("Zero angles round-trip correctly")
	{
		ufo::Vec3d angles(0.0, 0.0, 0.0);
		ufo::Quatd q(angles);
		auto       recovered = eulerAngles(q);
		REQUIRE(recovered[0] == Catch::Approx(angles[0]).margin(1e-10));
		REQUIRE(recovered[1] == Catch::Approx(angles[1]).margin(1e-10));
		REQUIRE(recovered[2] == Catch::Approx(angles[2]).margin(1e-10));
	}

	SECTION("Small angles round-trip correctly")
	{
		ufo::Vec3d angles(0.1, 0.2, 0.3);
		ufo::Quatd q(angles);
		auto       recovered = eulerAngles(q);
		REQUIRE(recovered[0] == Catch::Approx(angles[0]).epsilon(1e-10));
		REQUIRE(recovered[1] == Catch::Approx(angles[1]).epsilon(1e-10));
		REQUIRE(recovered[2] == Catch::Approx(angles[2]).epsilon(1e-10));
	}
}

/**************************************************************************************
|                                                                                     |
|                            Additional constructor tests                             |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Quatd] [converting constructor] Converting constructor from different type")
{
	SECTION("Quatf to Quatd explicit conversion")
	{
		ufo::Quatf qf(0.5f, 0.5f, 0.5f, 0.5f);
		ufo::Quatd qd(qf);
		REQUIRE(qd.w == Catch::Approx(0.5));
		REQUIRE(qd.x == Catch::Approx(0.5));
		REQUIRE(qd.y == Catch::Approx(0.5));
		REQUIRE(qd.z == Catch::Approx(0.5));
	}
}

TEST_CASE("[Quatd] [constructor] Rotation from u to v")
{
	SECTION("Rotation from x-axis to y-axis is 90 degrees around z")
	{
		ufo::Vec3d u(1.0, 0.0, 0.0);
		ufo::Vec3d v(0.0, 1.0, 0.0);
		ufo::Quatd q(u, v);
		auto       r = q * u;
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("Rotation from a vector to itself is identity")
	{
		ufo::Vec3d u(0.0, 0.0, 1.0);
		ufo::Quatd q(u, u);
		auto       r = q * u;
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Quatd] [constructor] Construct from 4x4 matrix")
{
	SECTION("4x4 identity matrix gives identity quaternion")
	{
		ufo::Quatd q(ufo::Mat4d::identity());
		REQUIRE(q.w == Catch::Approx(1.0));
		REQUIRE(q.x == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(q.y == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(q.z == Catch::Approx(0.0).margin(1e-10));
	}
}

/**************************************************************************************
|                                                                                     |
|                               Conversion operator tests                             |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Quatd] [operator Mat3d] Conversion to 3x3 matrix")
{
	SECTION("Identity quaternion gives identity matrix")
	{
		ufo::Quatd q(1.0, 0.0, 0.0, 0.0);
		auto       m = ufo::Mat3d(q);
		REQUIRE(m == ufo::Mat3d::identity());
	}

	SECTION("90-degree z-rotation matrix round-trip")
	{
		ufo::Quatd q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		auto       m = ufo::Mat3d(q);
		// Rotating (1,0,0) by 90° around z gives (0,1,0)
		ufo::Vec3d r = m * ufo::Vec3d(1.0, 0.0, 0.0);
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
	}
}

TEST_CASE("[Quatd] [operator Mat4d] Conversion to 4x4 matrix")
{
	SECTION("Identity quaternion gives identity 4x4 matrix")
	{
		ufo::Quatd q(1.0, 0.0, 0.0, 0.0);
		auto       m = ufo::Mat4d(q);
		REQUIRE(m == ufo::Mat4d::identity());
	}
}

/**************************************************************************************
|                                                                                     |
|                                   Capacity tests                                    |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Quatd] [size] Size is always 4") { REQUIRE(ufo::Quatd::size() == 4); }

/**************************************************************************************
|                                                                                     |
|                                   Operations tests                                  |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Quatd] [swap] Swap operation")
{
	ufo::Quatd q1(1.0, 2.0, 3.0, 4.0);
	ufo::Quatd q2(5.0, 6.0, 7.0, 8.0);
	ufo::swap(q1, q2);
	REQUIRE(q1.w == 5.0);
	REQUIRE(q1.x == 6.0);
	REQUIRE(q2.w == 1.0);
	REQUIRE(q2.x == 2.0);
}

/**************************************************************************************
|                                                                                     |
|                            Additional function tests                                |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Quatd] [normSquared] Squared norm")
{
	SECTION("normSquared of identity is 1")
	{
		REQUIRE(ufo::normSquared(ufo::Quatd(1.0, 0.0, 0.0, 0.0)) == Catch::Approx(1.0));
	}

	SECTION("normSquared of (1,1,1,1) is 4")
	{
		REQUIRE(ufo::normSquared(ufo::Quatd(1.0, 1.0, 1.0, 1.0)) == Catch::Approx(4.0));
	}
}

TEST_CASE("[Quatd] [cross] Quaternion cross product")
{
	SECTION("cross(id, q) == q")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		ufo::Quatd q(0.5, 0.5, 0.5, 0.5);
		auto       r = ufo::cross(id, q);
		REQUIRE(r.w == Catch::Approx(q.w));
		REQUIRE(r.x == Catch::Approx(q.x));
		REQUIRE(r.y == Catch::Approx(q.y));
		REQUIRE(r.z == Catch::Approx(q.z));
	}

	SECTION("cross(q1, q2) == q1 * q2")
	{
		ufo::Quatd i(0.0, 1.0, 0.0, 0.0);
		ufo::Quatd j(0.0, 0.0, 1.0, 0.0);
		auto       via_cross = ufo::cross(i, j);
		auto       via_mul   = i * j;
		REQUIRE(via_cross.w == Catch::Approx(via_mul.w).margin(1e-15));
		REQUIRE(via_cross.x == Catch::Approx(via_mul.x).margin(1e-15));
		REQUIRE(via_cross.y == Catch::Approx(via_mul.y).margin(1e-15));
		REQUIRE(via_cross.z == Catch::Approx(via_mul.z).margin(1e-15));
	}
}

TEST_CASE("[Quatd] [mix] Mix interpolation")
{
	ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
	ufo::Quatd q90z = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));

	SECTION("mix at t=0 returns first quaternion")
	{
		auto r = ufo::mix(id, q90z, 0.0);
		REQUIRE(r.w == Catch::Approx(id.w));
		REQUIRE(r.z == Catch::Approx(id.z).margin(1e-10));
	}

	SECTION("mix at t=1 returns second quaternion")
	{
		auto r = ufo::mix(id, q90z, 1.0);
		REQUIRE(r.w == Catch::Approx(q90z.w));
		REQUIRE(r.z == Catch::Approx(q90z.z));
	}
}

TEST_CASE("[Quatd] [lerp] Linear interpolation")
{
	ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
	ufo::Quatd q(0.0, 1.0, 0.0, 0.0);

	SECTION("lerp at t=0 returns first quaternion")
	{
		REQUIRE(ufo::lerp(id, q, 0.0) == id);
	}

	SECTION("lerp at t=1 returns second quaternion")
	{
		REQUIRE(ufo::lerp(id, q, 1.0) == q);
	}

	SECTION("lerp at t=0.5 is midpoint")
	{
		auto r = ufo::lerp(id, q, 0.5);
		REQUIRE(r.w == Catch::Approx(0.5));
		REQUIRE(r.x == Catch::Approx(0.5));
	}
}

TEST_CASE("[Quatd] [angle] Extract rotation angle")
{
	SECTION("Identity quaternion has angle 0")
	{
		REQUIRE(ufo::angle(ufo::Quatd(1.0, 0.0, 0.0, 0.0)) ==
		        Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("90-degree rotation has angle pi/2")
	{
		auto q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		REQUIRE(ufo::angle(q) == Catch::Approx(std::numbers::pi / 2.0));
	}
}

TEST_CASE("[Quatd] [axis] Extract rotation axis")
{
	SECTION("Identity quaternion returns default axis (0,0,1)")
	{
		auto a = ufo::axis(ufo::Quatd(1.0, 0.0, 0.0, 0.0));
		REQUIRE(a[2] == Catch::Approx(1.0));
	}

	SECTION("90-degree z-rotation axis is (0,0,1)")
	{
		auto q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		auto a = ufo::axis(q);
		REQUIRE(a[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(a[1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(a[2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Quatd] [roll pitch yaw] Individual Euler components")
{
	SECTION("Pure roll rotation")
	{
		// Roll = rotation around X
		double     expected = std::numbers::pi / 4.0;
		ufo::Quatd q(ufo::Vec3d(expected, 0.0, 0.0));
		REQUIRE(ufo::pitch(q) == Catch::Approx(expected).epsilon(1e-10));
	}

	SECTION("Pure yaw rotation")
	{
		// Yaw = rotation around Z
		double     expected = std::numbers::pi / 6.0;
		ufo::Quatd q(ufo::Vec3d(0.0, expected, 0.0));
		REQUIRE(ufo::yaw(q) == Catch::Approx(expected).epsilon(1e-10));
	}
}

TEST_CASE("[Quatd] [rotate] Append angle-axis rotation")
{
	SECTION("Appending zero rotation leaves quaternion unchanged")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		auto       r = ufo::rotate(id, 0.0, ufo::Vec3d(0.0, 0.0, 1.0));
		REQUIRE(r.w == Catch::Approx(1.0));
		REQUIRE(r.z == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("Appending 90-degree z rotation to identity gives 90-degree z quaternion")
	{
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		auto       r  = ufo::rotate(id, std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		auto expected = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		REQUIRE(r.w == Catch::Approx(expected.w));
		REQUIRE(r.z == Catch::Approx(expected.z));
	}
}

TEST_CASE("[Quatd] [v * q] Inverse rotation of vector")
{
	SECTION("v * identity = v")
	{
		ufo::Vec3d v(1.0, 2.0, 3.0);
		ufo::Quatd id(1.0, 0.0, 0.0, 0.0);
		auto       r = v * id;
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("v * q == inverse(q) * v")
	{
		ufo::Vec3d v(1.0, 0.0, 0.0);
		ufo::Quatd q  = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		auto       r1 = v * q;
		auto       r2 = ufo::inverse(q) * v;
		REQUIRE(r1[0] == Catch::Approx(r2[0]).margin(1e-10));
		REQUIRE(r1[1] == Catch::Approx(r2[1]).margin(1e-10));
		REQUIRE(r1[2] == Catch::Approx(r2[2]).margin(1e-10));
	}
}

TEST_CASE("[Quatd] [q * Vec4] Rotation of 4-vector")
{
	SECTION("q * Vec4 rotates xyz and preserves w")
	{
		ufo::Quatd q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
		ufo::Vec<4, double> v(1.0, 0.0, 0.0, 99.0);
		auto                r = q * v;
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[3] == Catch::Approx(99.0));
	}
}

TEST_CASE("[Quatd] [isnan isinf] NaN and Inf detection")
{
	SECTION("Identity quaternion has no NaN")
	{
		auto r = ufo::isnan(ufo::Quatd(1.0, 0.0, 0.0, 0.0));
		REQUIRE(!r[0]);
		REQUIRE(!r[1]);
		REQUIRE(!r[2]);
		REQUIRE(!r[3]);
	}

	SECTION("Identity quaternion has no Inf")
	{
		auto r = ufo::isinf(ufo::Quatd(1.0, 0.0, 0.0, 0.0));
		REQUIRE(!r[0]);
		REQUIRE(!r[1]);
		REQUIRE(!r[2]);
		REQUIRE(!r[3]);
	}

	SECTION("NaN in w component is detected")
	{
		ufo::Quatd q(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
		auto       r = ufo::isnan(q);
		REQUIRE(r[0]);
		REQUIRE(!r[1]);
	}

	SECTION("Inf in x component is detected")
	{
		ufo::Quatd q(1.0, std::numeric_limits<double>::infinity(), 0.0, 0.0);
		auto       r = ufo::isinf(q);
		REQUIRE(!r[0]);
		REQUIRE(r[1]);
	}
}
