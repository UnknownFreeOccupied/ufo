// UFO
#include <ufo/vision/color/rgb.hpp>
#include <ufo/vision/image.hpp>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using namespace ufo;

TEST_CASE("Image basic operations")
{
	SECTION("Construction and basic properties")
	{
		Image<float> img(10, 20, 1.0f);
		CHECK(img.rows() == 10);
		CHECK(img.cols() == 20);
		CHECK(img.size() == 200);
		CHECK(std::all_of(img.begin(), img.end(), [](float v) { return v == 1.0f; }));
	}

	SECTION("Resize and memory reuse")
	{
		Image<float> img(10, 10);
		auto*        data = img.data();
		img.resize(10, 10);
		CHECK(img.data() == data);  // Should reuse memory

		img.resize(5, 20);
		CHECK(img.data() == data);  // Should reuse memory (same size)

		img.resize(11, 10);
		CHECK(img.data() != data);  // Should reallocate
	}

	SECTION("Fill and clear")
	{
		Image<float> img(10, 10, 0.0f);
		img.fill(5.0f);
		CHECK(std::all_of(img.begin(), img.end(), [](float v) { return v == 5.0f; }));

		img.clear();
		CHECK(img.empty());
		CHECK(img.rows() == 0);
		CHECK(img.cols() == 0);
	}
}

TEST_CASE("Image transformations")
{
	Image<int> img(2, 3);
	img[0, 0] = 1;
	img[0, 1] = 2;
	img[0, 2] = 3;
	img[1, 0] = 4;
	img[1, 1] = 5;
	img[1, 2] = 6;

	SECTION("Transpose")
	{
		auto t = img.transposed();
		CHECK(t.rows() == 3);
		CHECK(t.cols() == 2);
		CHECK(t[0, 0] == 1);
		CHECK(t[0, 1] == 4);
		CHECK(t[1, 0] == 2);
		CHECK(t[1, 1] == 5);
		CHECK(t[2, 0] == 3);
		CHECK(t[2, 1] == 6);
	}

	SECTION("Flip Horizontal")
	{
		auto f = img;
		f.flipHorizontal();
		CHECK(f[0, 0] == 3);
		CHECK(f[0, 1] == 2);
		CHECK(f[0, 2] == 1);
		CHECK(f[1, 0] == 6);
		CHECK(f[1, 1] == 5);
		CHECK(f[1, 2] == 4);
	}

	SECTION("Flip Vertical")
	{
		auto f = img;
		f.flipVertical();
		CHECK(f[0, 0] == 4);
		CHECK(f[0, 1] == 5);
		CHECK(f[0, 2] == 6);
		CHECK(f[1, 0] == 1);
		CHECK(f[1, 1] == 2);
		CHECK(f[1, 2] == 3);
	}

	SECTION("Rotate 90 Clockwise")
	{
		auto r = img.rotate90(true);
		CHECK(r.rows() == 3);
		CHECK(r.cols() == 2);
		CHECK(r[0, 0] == 4);
		CHECK(r[0, 1] == 1);
		CHECK(r[1, 0] == 5);
		CHECK(r[1, 1] == 2);
		CHECK(r[2, 0] == 6);
		CHECK(r[2, 1] == 3);
	}
}

TEST_CASE("Image sampling")
{
	Image<float> img(2, 2);
	img[0, 0] = 0.0f;
	img[0, 1] = 1.0f;
	img[1, 0] = 1.0f;
	img[1, 1] = 2.0f;

	SECTION("Normalized sampling")
	{
		CHECK(img.sample(0.0f, 0.0f) == 0.0f);
		CHECK(img.sample(0.0f, 1.0f) == 1.0f);
		CHECK(img.sample(1.0f, 0.0f) == 1.0f);
		CHECK(img.sample(1.0f, 1.0f) == 2.0f);
		CHECK(img.sample(0.5f, 0.5f) == Catch::Approx(1.0f));
	}

	SECTION("Pixel-based sampling")
	{
		CHECK(img.samplePixel(0.0f, 0.0f) == 0.0f);
		CHECK(img.samplePixel(1.0f, 1.0f) == 2.0f);
		CHECK(img.samplePixel(0.5f, 0.5f) == Catch::Approx(1.0f));
	}
}

TEST_CASE("Image scaling")
{
	Image<float> img(2, 2);
	img[0, 0] = 10.0f;
	img[0, 1] = 20.0f;
	img[1, 0] = 30.0f;
	img[1, 1] = 40.0f;

	SECTION("Rescale up")
	{
		auto res = img.rescaled(3, 3);
		CHECK(res.rows() == 3);
		CHECK(res.cols() == 3);
		CHECK(res[0, 0] == 10.0f);
		CHECK(res[0, 2] == 20.0f);
		CHECK(res[1, 1] == Catch::Approx(25.0f));
		CHECK(res[2, 0] == 30.0f);
		CHECK(res[2, 2] == 40.0f);
	}

	SECTION("Upscale factor")
	{
		auto f = img;
		f.upscale(2.0f);
		CHECK(f.rows() == 4);
		CHECK(f.cols() == 4);
		CHECK(f[0, 0] == 10.0f);
		CHECK(f[3, 3] == 40.0f);
	}

	SECTION("Downscale factor")
	{
		Image<float> big(4, 4, 1.0f);
		big.downscale(2.0f);
		CHECK(big.rows() == 2);
		CHECK(big.cols() == 2);
		big[0, 0] = 5.0f;
		CHECK(big[0, 0] == 5.0f);
	}
}
