// UFO
#include <ufo/math/transform.hpp>

// STL
#include <numbers>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[Transform3d] [constructor] Default constructor")
{
	ufo::Transform3d t;

	SECTION("Rotation defaults to identity matrix")
	{
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				REQUIRE(t.rotation[r][c] == (r == c ? 1.0 : 0.0));
	}

	SECTION("Translation defaults to zero vector")
	{
		REQUIRE(t.translation[0] == 0.0);
		REQUIRE(t.translation[1] == 0.0);
		REQUIRE(t.translation[2] == 0.0);
	}
}

TEST_CASE("[Transform3d] [constructor] Construct from quaternion and translation")
{
	// 90-degree rotation around z-axis
	ufo::Quatd       q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
	ufo::Vec3d       tr(1.0, 2.0, 3.0);
	ufo::Transform3d t(q, tr);

	SECTION("Translation is set correctly")
	{
		REQUIRE(t.translation[0] == Catch::Approx(1.0));
		REQUIRE(t.translation[1] == Catch::Approx(2.0));
		REQUIRE(t.translation[2] == Catch::Approx(3.0));
	}

	SECTION("Rotation matches quaternion-derived matrix")
	{
		auto expected = ufo::Mat3d(q);
		REQUIRE(t.rotation == expected);
	}
}

TEST_CASE("[Transform3d] [constructor] Construct from homogeneous matrix")
{
	// Build a 4x4 matrix with known rotation and translation
	auto hom  = ufo::Mat4d::identity();
	hom[0][3] = 5.0;
	hom[1][3] = 6.0;
	hom[2][3] = 7.0;
	ufo::Transform3d t(hom);

	SECTION("Translation extracted from last column")
	{
		REQUIRE(t.translation[0] == Catch::Approx(5.0));
		REQUIRE(t.translation[1] == Catch::Approx(6.0));
		REQUIRE(t.translation[2] == Catch::Approx(7.0));
	}

	SECTION("Rotation extracted from upper-left block")
	{
		REQUIRE(t.rotation[0][0] == Catch::Approx(1.0));
		REQUIRE(t.rotation[1][1] == Catch::Approx(1.0));
		REQUIRE(t.rotation[2][2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Transform3d] [operator==] Equality operator")
{
	ufo::Transform3d t1;
	ufo::Transform3d t2;
	REQUIRE(t1 == t2);
}

TEST_CASE("[Transform3d] [operator!=] Inequality operator")
{
	ufo::Transform3d t1;
	ufo::Transform3d t2;
	t2.translation[0] = 1.0;
	REQUIRE(t1 != t2);
}

TEST_CASE("[Transform3d] [operator()] Apply transform to vector")
{
	SECTION("Identity transform leaves vector unchanged")
	{
		ufo::Transform3d t;
		ufo::Vec3d       v(1.0, 2.0, 3.0);
		auto             r = t(v);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("Pure translation shifts vector")
	{
		ufo::Transform3d t;
		t.translation = ufo::Vec3d(1.0, 2.0, 3.0);
		ufo::Vec3d v(0.0, 0.0, 0.0);
		auto       r = t(v);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
		REQUIRE(r[2] == Catch::Approx(3.0));
	}

	SECTION("90-degree rotation around z-axis rotates x to y")
	{
		ufo::Quatd       q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0, 0, 1));
		ufo::Transform3d t(q);
		ufo::Vec3d       v(1.0, 0.0, 0.0);
		auto             r = t(v);
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("Rotation and translation are applied correctly")
	{
		// 90-degree rotation around z + translation (1,0,0)
		// Rotating (1,0,0) by 90° around z gives (0,1,0), then +translation = (1,1,0)
		ufo::Quatd       q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0, 0, 1));
		ufo::Transform3d t(q, ufo::Vec3d(1.0, 0.0, 0.0));
		ufo::Vec3d       v(1.0, 0.0, 0.0);
		auto             r = t(v);
		REQUIRE(r[0] == Catch::Approx(1.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
		REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
	}
}

TEST_CASE("[Transform3d] [operator*] Composition operator")
{
	SECTION("Identity * identity = identity")
	{
		ufo::Transform3d id;
		REQUIRE((id * id) == id);
	}

	SECTION("Pure translations compose by addition")
	{
		ufo::Transform3d t1;
		ufo::Transform3d t2;
		t1.translation = ufo::Vec3d(1.0, 0.0, 0.0);
		t2.translation = ufo::Vec3d(0.0, 2.0, 0.0);
		auto r         = t1 * t2;
		REQUIRE(r.translation[0] == Catch::Approx(1.0));
		REQUIRE(r.translation[1] == Catch::Approx(2.0));
		REQUIRE(r.translation[2] == Catch::Approx(0.0));
	}

	SECTION("Composition is associative")
	{
		ufo::Transform3d t1, t2, t3;
		t1.translation = ufo::Vec3d(1.0, 0.0, 0.0);
		t2.translation = ufo::Vec3d(0.0, 1.0, 0.0);
		t3.translation = ufo::Vec3d(0.0, 0.0, 1.0);
		REQUIRE((t1 * t2) * t3 == t1 * (t2 * t3));
	}

	SECTION("Two 90-degree rotations around z equals 180-degree rotation")
	{
		ufo::Quatd       q90  = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0, 0, 1));
		ufo::Quatd       q180 = ufo::angleAxis(std::numbers::pi, ufo::Vec3d(0, 0, 1));
		ufo::Transform3d r90(q90);
		ufo::Transform3d r180(q180);
		auto             composed = r90 * r90;
		// Check that (1,0,0) is rotated to (-1,0,0)
		ufo::Vec3d x(1.0, 0.0, 0.0);
		auto       result   = composed(x);
		auto       expected = r180(x);
		REQUIRE(result[0] == Catch::Approx(expected[0]).margin(1e-10));
		REQUIRE(result[1] == Catch::Approx(expected[1]).margin(1e-10));
		REQUIRE(result[2] == Catch::Approx(expected[2]).margin(1e-10));
	}
}

TEST_CASE("[Transform3d] [inverse] Inverse transform")
{
	SECTION("Inverse of identity is identity")
	{
		ufo::Transform3d id;
		REQUIRE(inverse(id) == id);
	}

	SECTION("T * inverse(T) = identity for pure translation")
	{
		ufo::Transform3d t;
		t.translation = ufo::Vec3d(3.0, 4.0, 5.0);
		auto composed = t * inverse(t);
		REQUIRE(composed.translation[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(composed.translation[1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(composed.translation[2] == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("inverse(T) * T = identity for pure translation")
	{
		ufo::Transform3d t;
		t.translation = ufo::Vec3d(1.0, 2.0, 3.0);
		auto composed = inverse(t) * t;
		REQUIRE(composed.translation[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(composed.translation[1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(composed.translation[2] == Catch::Approx(0.0).margin(1e-10));
	}

	SECTION("Inverse undoes rotation")
	{
		ufo::Quatd q = ufo::angleAxis(std::numbers::pi / 3.0, normalize(ufo::Vec3d(1, 1, 0)));
		ufo::Transform3d t(q);
		ufo::Vec3d       v(1.0, 0.0, 0.0);
		auto             result = inverse(t)(t(v));
		REQUIRE(result[0] == Catch::Approx(v[0]).margin(1e-10));
		REQUIRE(result[1] == Catch::Approx(v[1]).margin(1e-10));
		REQUIRE(result[2] == Catch::Approx(v[2]).margin(1e-10));
	}
}

TEST_CASE("[Transform2d] [theta] Extract rotation angle")
{
	SECTION("Default transform has theta = 0")
	{
		ufo::Transform2d t;
		REQUIRE(t.theta() == Catch::Approx(0.0));
	}

	SECTION("45-degree rotation has theta = pi/4")
	{
		ufo::Transform2d t(std::numbers::pi / 4.0);
		REQUIRE(t.theta() == Catch::Approx(std::numbers::pi / 4.0));
	}

	SECTION("90-degree rotation has theta = pi/2")
	{
		ufo::Transform2d t(std::numbers::pi / 2.0);
		REQUIRE(t.theta() == Catch::Approx(std::numbers::pi / 2.0));
	}

	SECTION("Negative angle round-trips correctly")
	{
		double           angle = -std::numbers::pi / 6.0;
		ufo::Transform2d t(angle);
		REQUIRE(t.theta() == Catch::Approx(angle));
	}
}

/**************************************************************************************
|                                                                                     |
|                       Additional Transform2d tests                                  |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Transform2d] [constructor] Default constructor")
{
	ufo::Transform2d t;
	REQUIRE(t.rotation == ufo::Mat2d::identity());
	REQUIRE(t.translation[0] == 0.0);
	REQUIRE(t.translation[1] == 0.0);
}

TEST_CASE("[Transform2d] [constructor] Angle and translation")
{
	ufo::Transform2d t(std::numbers::pi / 2.0, ufo::Vec2d(3.0, 4.0));
	REQUIRE(t.theta() == Catch::Approx(std::numbers::pi / 2.0));
	REQUIRE(t.translation[0] == Catch::Approx(3.0));
	REQUIRE(t.translation[1] == Catch::Approx(4.0));
}

TEST_CASE("[Transform2d] [operator()] Apply 2D transform to vector")
{
	SECTION("Identity transform leaves vector unchanged")
	{
		ufo::Transform2d t;
		ufo::Vec2d       v(3.0, 4.0);
		auto             r = t(v);
		REQUIRE(r[0] == Catch::Approx(3.0));
		REQUIRE(r[1] == Catch::Approx(4.0));
	}

	SECTION("Pure translation shifts vector")
	{
		ufo::Transform2d t;
		t.translation = ufo::Vec2d(1.0, 2.0);
		ufo::Vec2d v(0.0, 0.0);
		auto       r = t(v);
		REQUIRE(r[0] == Catch::Approx(1.0));
		REQUIRE(r[1] == Catch::Approx(2.0));
	}

	SECTION("90-degree rotation rotates x to y")
	{
		ufo::Transform2d t(std::numbers::pi / 2.0);
		ufo::Vec2d       v(1.0, 0.0);
		auto             r = t(v);
		REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(r[1] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Transform2d] [operator*] 2D composition")
{
	SECTION("Two 45-degree rotations compose to 90 degrees")
	{
		ufo::Transform2d t45(std::numbers::pi / 4.0);
		auto             t90 = t45 * t45;
		REQUIRE(t90.theta() == Catch::Approx(std::numbers::pi / 2.0).margin(1e-10));
	}
}

/**************************************************************************************
|                                                                                     |
|                       Additional Transform3d tests                                  |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Transform3d] [constructor] Rotation matrix only")
{
	auto m = ufo::Mat3d(ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0)));
	ufo::Transform3d t(m);
	REQUIRE(t.translation[0] == 0.0);
	REQUIRE(t.translation[1] == 0.0);
	REQUIRE(t.translation[2] == 0.0);
	REQUIRE(t.rotation == m);
}

TEST_CASE("[Transform3d] [converting constructor] From Transform3f")
{
	ufo::Transform3f tf;
	tf.translation = ufo::Vec3f(1.0f, 2.0f, 3.0f);
	ufo::Transform3d td(tf);
	REQUIRE(td.translation[0] == Catch::Approx(1.0));
	REQUIRE(td.translation[1] == Catch::Approx(2.0));
	REQUIRE(td.translation[2] == Catch::Approx(3.0));
}

TEST_CASE("[Transform3d] [assignment] Converting assignment from Transform3f")
{
	ufo::Transform3f tf;
	tf.translation = ufo::Vec3f(5.0f, 6.0f, 7.0f);
	ufo::Transform3d td;
	td = ufo::Transform3d(tf);
	REQUIRE(td.translation[0] == Catch::Approx(5.0));
	REQUIRE(td.translation[1] == Catch::Approx(6.0));
	REQUIRE(td.translation[2] == Catch::Approx(7.0));
}

TEST_CASE("[Transform3d] [conversion] To homogeneous matrix")
{
	ufo::Quatd       q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
	ufo::Transform3d t(q, ufo::Vec3d(1.0, 2.0, 3.0));
	auto             hom = ufo::Mat4d(t);

	SECTION("Translation is in last column")
	{
		REQUIRE(hom[0][3] == Catch::Approx(1.0));
		REQUIRE(hom[1][3] == Catch::Approx(2.0));
		REQUIRE(hom[2][3] == Catch::Approx(3.0));
	}

	SECTION("Bottom-right element is 1") { REQUIRE(hom[3][3] == Catch::Approx(1.0)); }

	SECTION("Upper-left 3x3 matches rotation matrix")
	{
		auto expected = ufo::Mat3d(q);
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				REQUIRE(hom[r][c] == Catch::Approx(expected[r][c]).margin(1e-10));
	}
}

TEST_CASE("[Transform3d] [conversion] To quaternion")
{
	ufo::Quatd q_in = ufo::angleAxis(std::numbers::pi / 3.0, ufo::Vec3d(0.0, 1.0, 0.0));
	ufo::Transform3d t(q_in);
	ufo::Quatd       q_out = ufo::Quatd(t);
	REQUIRE(q_out.w == Catch::Approx(q_in.w).margin(1e-10));
	REQUIRE(q_out.x == Catch::Approx(q_in.x).margin(1e-10));
	REQUIRE(q_out.y == Catch::Approx(q_in.y).margin(1e-10));
	REQUIRE(q_out.z == Catch::Approx(q_in.z).margin(1e-10));
}

TEST_CASE("[Transform3d] [operator*=] Compound composition")
{
	ufo::Transform3d t1;
	t1.translation = ufo::Vec3d(1.0, 0.0, 0.0);
	ufo::Transform3d t2;
	t2.translation = ufo::Vec3d(0.0, 2.0, 0.0);
	t1 *= t2;
	REQUIRE(t1.translation[0] == Catch::Approx(1.0));
	REQUIRE(t1.translation[1] == Catch::Approx(2.0));
	REQUIRE(t1.translation[2] == Catch::Approx(0.0));
}

TEST_CASE("[Transform3d] [operator* Vec] Transform * vector free operator")
{
	ufo::Quatd       q = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
	ufo::Transform3d t(q, ufo::Vec3d(0.0, 0.0, 0.0));
	ufo::Vec3d       v(1.0, 0.0, 0.0);
	auto             r = t * v;
	REQUIRE(r[0] == Catch::Approx(0.0).margin(1e-10));
	REQUIRE(r[1] == Catch::Approx(1.0));
	REQUIRE(r[2] == Catch::Approx(0.0).margin(1e-10));
}

TEST_CASE("[Transform3d] [operator* Quat] Transform * quaternion free operator")
{
	ufo::Quatd q90 = ufo::angleAxis(std::numbers::pi / 2.0, ufo::Vec3d(0.0, 0.0, 1.0));
	ufo::Transform3d t(q90);
	ufo::Quatd       id(1.0, 0.0, 0.0, 0.0);
	auto             r = t * id;
	// Applying rotation transform to identity quat should give q90
	REQUIRE(r.w == Catch::Approx(q90.w).margin(1e-10));
	REQUIRE(r.z == Catch::Approx(q90.z).margin(1e-10));
}

TEST_CASE("[Transform3d] [inverse] Full rotation + translation inverse")
{
	ufo::Quatd       q = ufo::angleAxis(std::numbers::pi / 4.0, ufo::Vec3d(0.0, 1.0, 0.0));
	ufo::Vec3d       tr(1.0, 2.0, 3.0);
	ufo::Transform3d t(q, tr);
	ufo::Vec3d       v(4.0, 5.0, 6.0);

	SECTION("inverse(t)(t(v)) == v")
	{
		auto result = ufo::inverse(t)(t(v));
		REQUIRE(result[0] == Catch::Approx(v[0]).margin(1e-10));
		REQUIRE(result[1] == Catch::Approx(v[1]).margin(1e-10));
		REQUIRE(result[2] == Catch::Approx(v[2]).margin(1e-10));
	}

	SECTION("t * inverse(t) has zero translation")
	{
		auto composed = t * ufo::inverse(t);
		REQUIRE(composed.translation[0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(composed.translation[1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(composed.translation[2] == Catch::Approx(0.0).margin(1e-10));
	}
}
