// UFO
#include <ufo/vision/camera.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("Perspective Camera Intrinsics")
{
	PerspectiveIntrinsics intrinsics;
	intrinsics.rows         = 480;
	intrinsics.cols         = 640;
	intrinsics.vertical_fov = 1.0f;  // radians
	intrinsics.cx           = 320.0f;
	intrinsics.cy           = 240.0f;

	CHECK(intrinsics.aspect() == Catch::Approx(640.0f / 480.0f));

	SECTION("Focal length derivation")
	{
		PerspectiveCamera cam;
		cam.intrinsics = intrinsics;
		cam.near_clip  = 0.1f;
		cam.far_clip   = 100.0f;

		auto proj = cam.projection();
		// m[1][1] should be 2 * fy / rows
		// fy = rows / (2 * tan(fovy/2))
		// m[1][1] = 1 / tan(fovy/2)
		CHECK(proj[1][1] == Catch::Approx(1.0f / std::tan(0.5f)).margin(0.01));
	}
}

TEST_CASE("Orthogonal Camera Intrinsics")
{
	OrthogonalIntrinsics intrinsics;
	intrinsics.rows   = 480;
	intrinsics.cols   = 640;
	intrinsics.width  = 6.4f;  // meters
	intrinsics.height = 4.8f;  // meters

	CHECK(intrinsics.aspect() == Catch::Approx(640.0f / 480.0f));

	SECTION("Projection matrix")
	{
		OrthogonalCamera cam;
		cam.intrinsics = intrinsics;
		cam.near_clip  = 0.1f;
		cam.far_clip   = 100.0f;

		auto proj = cam.projection();
		// m[0][0] = 2 / width
		CHECK(proj[0][0] == Catch::Approx(2.0f / 6.4f));
		// m[1][1] = 2 / height
		CHECK(proj[1][1] == Catch::Approx(2.0f / 4.8f));
	}
}

TEST_CASE("Camera Pose and lookAt")
{
	PerspectiveCamera cam;
	cam.intrinsics.rows         = 100;
	cam.intrinsics.cols         = 100;
	cam.intrinsics.vertical_fov = 1.0f;
	cam.intrinsics.cx           = 50.0f;
	cam.intrinsics.cy           = 50.0f;
	cam.near_clip               = 0.1f;
	cam.far_clip                = 100.0f;

	SECTION("Default pose")
	{
		cam.pose = Transform3f{};
		auto v   = cam.view();
		CHECK(v[0][0] == 1.0f);
		CHECK(v[1][1] == 1.0f);
		CHECK(v[2][2] == 1.0f);
		CHECK(v[3][3] == 1.0f);
	}

	SECTION("lookAt OpenCV style (+Z forward)")
	{
		Vec3f eye(0.0f, 0.0f, 0.0f);
		Vec3f target(0.0f, 0.0f, 10.0f);
		cam.lookAt(eye, target, Vec3f(0.0f, -1.0f, 0.0f));

		// Pose should be identity (looking along +Z world)
		auto p = cam.pose;
		CHECK(p.translation.x() == Catch::Approx(0.0f));
		CHECK(p.translation.y() == Catch::Approx(0.0f));
		CHECK(p.translation.z() == Catch::Approx(0.0f));

		// Check axes
		auto v = cam.view();
		// Forward (+Z cam) is +Z world
		CHECK(v[0][2] == Catch::Approx(0.0f));
		CHECK(v[1][2] == Catch::Approx(0.0f));
		CHECK(v[2][2] == Catch::Approx(1.0f));
	}
}

TEST_CASE("Camera Ray Generation")
{
	SECTION("Perspective center ray")
	{
		PerspectiveCamera cam;
		cam.intrinsics.rows         = 100;
		cam.intrinsics.cols         = 100;
		cam.intrinsics.vertical_fov = 1.0f;
		cam.intrinsics.cx           = 50.0f;
		cam.intrinsics.cy           = 50.0f;
		cam.near_clip               = 0.1f;
		cam.far_clip                = 100.0f;
		cam.pose                    = Transform3f{};

		auto rays = cam.rays();
		auto r    = rays[50, 50];
		// Pixel (50, 50) is the center of the image.
		// dx = (50 + 0.5 - 50) / fx = 0.5 / fx
		// dy = (50 + 0.5 - 50) / fy = 0.5 / fy
		// dir_cam = normalize([dx, dy, 1])
		// For an identity pose, that should give [0, 0, 1] (with a small margin for the 0.5
		// pixel offset).
		CHECK(r.direction.x() == Catch::Approx(0.0f).margin(0.01));
		CHECK(r.direction.y() == Catch::Approx(0.0f).margin(0.01));
		CHECK(r.direction.z() == Catch::Approx(1.0f).margin(0.01));
	}

	SECTION("Orthogonal rays")
	{
		OrthogonalCamera cam;
		cam.intrinsics.rows   = 100;
		cam.intrinsics.cols   = 100;
		cam.intrinsics.width  = 10.0f;
		cam.intrinsics.height = 10.0f;
		cam.near_clip         = 0.1f;
		cam.far_clip          = 100.0f;
		cam.pose              = Transform3f{};

		auto rays = cam.rays();
		// All rays should point along +Z
		auto r1 = rays[0, 0];
		auto r2 = rays[50, 50];

		CHECK(r1.direction.x() == Catch::Approx(0.0f));
		CHECK(r1.direction.y() == Catch::Approx(0.0f));
		CHECK(r1.direction.z() == Catch::Approx(1.0f));

		CHECK(r2.direction.x() == Catch::Approx(0.0f));
		CHECK(r2.direction.y() == Catch::Approx(0.0f));
		CHECK(r2.direction.z() == Catch::Approx(1.0f));

		// Origin should vary.
		// vx = ((x + 0.5f) / rays.cols()) * 2.0f - 1.0f;
		// vy = ((y + 0.5f) / rays.rows()) * 2.0f - 1.0f;
		// For (0,0): vx = (0.5/100)*2 - 1 = 0.01 - 1 = -0.99
		// x = vx * (width/2) = -0.99 * 5 = -4.95
		CHECK(r1.origin.x() == Catch::Approx(-4.95f));
		CHECK(r1.origin.y() == Catch::Approx(-4.95f));
	}
}
