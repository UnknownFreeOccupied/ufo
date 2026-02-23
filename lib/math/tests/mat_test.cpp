// UFO
#include <ufo/math/mat.hpp>

// STL
#include <stdexcept>

// Catch2
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("[Mat3d] [zeros] Zero factory")
{
	auto m = ufo::Mat3d::zeros();
	for (std::size_t r = 0; r < 3; ++r)
		for (std::size_t c = 0; c < 3; ++c) REQUIRE(m[r][c] == 0.0);
}

TEST_CASE("[Mat3d] [identity] Identity factory")
{
	auto m = ufo::Mat3d::identity();
	for (std::size_t r = 0; r < 3; ++r)
		for (std::size_t c = 0; c < 3; ++c) REQUIRE(m[r][c] == (r == c ? 1.0 : 0.0));
}

TEST_CASE("[Mat3d] [ones] Ones factory")
{
	auto m = ufo::Mat3d::ones();
	for (std::size_t r = 0; r < 3; ++r)
		for (std::size_t c = 0; c < 3; ++c) REQUIRE(m[r][c] == 1.0);
}

TEST_CASE("[Mat3d] [operator==] Equality operator")
{
	auto a = ufo::Mat3d::identity();
	auto b = ufo::Mat3d::identity();
	REQUIRE(a == b);
}

TEST_CASE("[Mat3d] [operator!=] Inequality operator")
{
	auto a = ufo::Mat3d::identity();
	auto b = ufo::Mat3d::zeros();
	REQUIRE(a != b);
}

TEST_CASE("[Mat3d] [operator+] Addition operator")
{
	auto   id     = ufo::Mat3d::identity();
	double scalar = 1.0;

	SECTION("matrix + matrix")
	{
		auto r = id + id;
		REQUIRE(r[0][0] == 2.0);
		REQUIRE(r[1][1] == 2.0);
		REQUIRE(r[2][2] == 2.0);
		REQUIRE(r[0][1] == 0.0);
	}

	SECTION("matrix + scalar")
	{
		auto r = id + scalar;
		REQUIRE(r[0][0] == 2.0);
		REQUIRE(r[0][1] == 1.0);
		REQUIRE(r[1][0] == 1.0);
	}

	SECTION("scalar + matrix")
	{
		auto r = scalar + id;
		REQUIRE(r[0][0] == 2.0);
		REQUIRE(r[0][1] == 1.0);
	}
}

TEST_CASE("[Mat3d] [operator-] Subtraction operator")
{
	auto   id     = ufo::Mat3d::identity();
	double scalar = 1.0;

	SECTION("matrix - matrix (identity - identity = zero)")
	{
		auto r = id - id;
		for (std::size_t i = 0; i < 3; ++i)
			for (std::size_t j = 0; j < 3; ++j) REQUIRE(r[i][j] == 0.0);
	}

	SECTION("matrix - scalar")
	{
		auto r = id - scalar;
		REQUIRE(r[0][0] == 0.0);
		REQUIRE(r[0][1] == -1.0);
	}

	SECTION("Unary negation")
	{
		auto r = -id;
		REQUIRE(r[0][0] == -1.0);
		REQUIRE(r[0][1] == 0.0);
	}
}

TEST_CASE("[Mat3d] [operator*] Multiplication operator")
{
	auto id = ufo::Mat3d::identity();
	// diagonal(1,2,3)
	ufo::Mat3d diag(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);

	SECTION("identity * identity = identity")
	{
		auto r = id * id;
		REQUIRE(r == id);
	}

	SECTION("identity * matrix = matrix")
	{
		auto r = id * diag;
		REQUIRE(r == diag);
	}

	SECTION("diagonal * diagonal (element-wise product on diagonal)")
	{
		// diag(1,2,3) * diag(1,2,3) = diag(1,4,9)
		auto r = diag * diag;
		REQUIRE(r[0][0] == 1.0);
		REQUIRE(r[1][1] == 4.0);
		REQUIRE(r[2][2] == 9.0);
		REQUIRE(r[0][1] == 0.0);
	}

	SECTION("matrix * scalar")
	{
		auto r = diag * 2.0;
		REQUIRE(r[0][0] == 2.0);
		REQUIRE(r[1][1] == 4.0);
		REQUIRE(r[2][2] == 6.0);
	}

	SECTION("scalar * matrix")
	{
		auto r = 2.0 * diag;
		REQUIRE(r[0][0] == 2.0);
		REQUIRE(r[1][1] == 4.0);
		REQUIRE(r[2][2] == 6.0);
	}

	SECTION("matrix * vector")
	{
		// diag(1,2,3) * (1,1,1) = (1,2,3)
		ufo::Vec3d v(1.0, 1.0, 1.0);
		auto       r = diag * v;
		REQUIRE(r[0] == 1.0);
		REQUIRE(r[1] == 2.0);
		REQUIRE(r[2] == 3.0);
	}

	SECTION("identity * vector = vector")
	{
		ufo::Vec3d v(4.0, 5.0, 6.0);
		auto       r = id * v;
		REQUIRE(r[0] == 4.0);
		REQUIRE(r[1] == 5.0);
		REQUIRE(r[2] == 6.0);
	}
}

TEST_CASE("[Mat3d] [operator+=] Compound addition operator")
{
	auto id = ufo::Mat3d::identity();

	SECTION("matrix += matrix")
	{
		id += ufo::Mat3d::identity();
		REQUIRE(id[0][0] == 2.0);
		REQUIRE(id[1][1] == 2.0);
		REQUIRE(id[0][1] == 0.0);
	}

	SECTION("matrix += scalar")
	{
		id += 1.0;
		REQUIRE(id[0][0] == 2.0);
		REQUIRE(id[0][1] == 1.0);
	}
}

TEST_CASE("[Mat3d] [operator-=] Compound subtraction operator")
{
	auto id = ufo::Mat3d::identity();

	SECTION("matrix -= matrix (becomes zero)")
	{
		id -= ufo::Mat3d::identity();
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c) REQUIRE(id[r][c] == 0.0);
	}

	SECTION("matrix -= scalar")
	{
		id -= 1.0;
		REQUIRE(id[0][0] == 0.0);
		REQUIRE(id[0][1] == -1.0);
	}
}

TEST_CASE("[Mat3d] [operator*=] Compound multiplication operator")
{
	ufo::Mat3d diag(2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0);

	SECTION("matrix *= matrix")
	{
		auto m = diag;
		m *= diag;
		REQUIRE(m[0][0] == 4.0);
		REQUIRE(m[1][1] == 9.0);
		REQUIRE(m[2][2] == 16.0);
	}

	SECTION("matrix *= scalar")
	{
		auto m = diag;
		m *= 2.0;
		REQUIRE(m[0][0] == 4.0);
		REQUIRE(m[1][1] == 6.0);
		REQUIRE(m[2][2] == 8.0);
	}
}

TEST_CASE("[Mat3d] [operator[]] Subscript operator")
{
	ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
	REQUIRE(m[0][0] == 1.0);
	REQUIRE(m[0][1] == 2.0);
	REQUIRE(m[0][2] == 3.0);
	REQUIRE(m[1][0] == 4.0);
	REQUIRE(m[1][1] == 5.0);
	REQUIRE(m[2][2] == 9.0);

	ufo::Mat3d const cm(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
	REQUIRE(cm[0][0] == 1.0);
	REQUIRE(cm[0][1] == 0.0);
}

TEST_CASE("[Mat3d] [at] Bounds-checked access")
{
	ufo::Mat3d m = ufo::Mat3d::identity();

	SECTION("Valid access")
	{
		REQUIRE(m.at(0, 0) == 1.0);
		REQUIRE(m.at(2, 2) == 1.0);
		REQUIRE(m.at(0, 1) == 0.0);
	}

	SECTION("Out-of-bounds access throws")
	{
		REQUIRE_THROWS_AS(m.at(3, 0), std::out_of_range);
		REQUIRE_THROWS_AS(m.at(0, 3), std::out_of_range);
	}
}

TEST_CASE("[Mat3d] [row] Row accessor")
{
	ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

	SECTION("First row")
	{
		auto r = m.row(0);
		REQUIRE(r[0] == 1.0);
		REQUIRE(r[1] == 2.0);
		REQUIRE(r[2] == 3.0);
	}

	SECTION("Last row")
	{
		auto r = m.row(2);
		REQUIRE(r[0] == 7.0);
		REQUIRE(r[1] == 8.0);
		REQUIRE(r[2] == 9.0);
	}
}

TEST_CASE("[Mat3d] [col] Column accessor")
{
	ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

	SECTION("First column")
	{
		auto c = m.col(0);
		REQUIRE(c[0] == 1.0);
		REQUIRE(c[1] == 4.0);
		REQUIRE(c[2] == 7.0);
	}

	SECTION("Last column")
	{
		auto c = m.col(2);
		REQUIRE(c[0] == 3.0);
		REQUIRE(c[1] == 6.0);
		REQUIRE(c[2] == 9.0);
	}
}

TEST_CASE("[Mat3d] [trace] Trace")
{
	SECTION("Trace of identity is 3")
	{
		REQUIRE(ufo::Mat3d::identity().trace() == Catch::Approx(3.0));
	}

	SECTION("Trace of zero matrix is 0")
	{
		REQUIRE(ufo::Mat3d::zeros().trace() == Catch::Approx(0.0));
	}

	SECTION("Trace of diagonal matrix")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
		REQUIRE(m.trace() == Catch::Approx(6.0));
	}

	SECTION("Trace ignores off-diagonal elements")
	{
		ufo::Mat3d m(1.0, 9.0, 9.0, 9.0, 2.0, 9.0, 9.0, 9.0, 3.0);
		REQUIRE(m.trace() == Catch::Approx(6.0));
	}
}

TEST_CASE("[Mat3d] [transpose]")
{
	SECTION("Transpose of identity is identity")
	{
		auto t = transpose(ufo::Mat3d::identity());
		REQUIRE(t == ufo::Mat3d::identity());
	}

	SECTION("Transpose swaps off-diagonal elements")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		auto       t = transpose(m);
		REQUIRE(t[0][0] == 1.0);
		REQUIRE(t[0][1] == 4.0);
		REQUIRE(t[0][2] == 7.0);
		REQUIRE(t[1][0] == 2.0);
		REQUIRE(t[1][1] == 5.0);
		REQUIRE(t[2][0] == 3.0);
	}

	SECTION("Double transpose returns original matrix")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		REQUIRE(transpose(transpose(m)) == m);
	}
}

TEST_CASE("[Mat3d] [determinant]")
{
	SECTION("Determinant of identity is 1")
	{
		REQUIRE(determinant(ufo::Mat3d::identity()) == Catch::Approx(1.0));
	}

	SECTION("Determinant of zero matrix is 0")
	{
		REQUIRE(determinant(ufo::Mat3d::zeros()) == Catch::Approx(0.0));
	}

	SECTION("Determinant of diagonal matrix is product of diagonal")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
		REQUIRE(determinant(m) == Catch::Approx(6.0));
	}

	SECTION("Determinant of singular matrix is 0")
	{
		// Rows are linearly dependent
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		REQUIRE(determinant(m) == Catch::Approx(0.0).margin(1e-10));
	}
}

TEST_CASE("[Mat3d] [inverse]")
{
	SECTION("Inverse of identity is identity")
	{
		auto inv = inverse(ufo::Mat3d::identity());
		REQUIRE(inv == ufo::Mat3d::identity());
	}

	SECTION("Inverse of diagonal matrix")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0);
		auto       inv = inverse(m);
		REQUIRE(inv[0][0] == Catch::Approx(1.0));
		REQUIRE(inv[1][1] == Catch::Approx(0.5));
		REQUIRE(inv[2][2] == Catch::Approx(0.25));
	}

	SECTION("M * inverse(M) = identity")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0);
		auto       id = m * inverse(m);
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				REQUIRE(id[r][c] == Catch::Approx(r == c ? 1.0 : 0.0).margin(1e-10));
	}

	SECTION("inverse(M) * M = identity")
	{
		ufo::Mat3d m(2.0, 1.0, 0.0, 0.0, 3.0, 1.0, 0.0, 0.0, 4.0);
		auto       id = inverse(m) * m;
		for (std::size_t r = 0; r < 3; ++r)
			for (std::size_t c = 0; c < 3; ++c)
				REQUIRE(id[r][c] == Catch::Approx(r == c ? 1.0 : 0.0).margin(1e-10));
	}
}

TEST_CASE("[Mat3d] [isDiagonal]")
{
	SECTION("Identity matrix is diagonal") { REQUIRE(ufo::Mat3d::identity().isDiagonal()); }

	SECTION("Zero matrix is diagonal") { REQUIRE(ufo::Mat3d::zeros().isDiagonal()); }

	SECTION("Diagonal matrix is diagonal")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
		REQUIRE(m.isDiagonal());
	}

	SECTION("Non-diagonal matrix is not diagonal")
	{
		ufo::Mat3d m(1.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
		REQUIRE(!m.isDiagonal());
	}
}

TEST_CASE("[Mat3d] [isSymmetric]")
{
	SECTION("Identity matrix is symmetric")
	{
		REQUIRE(ufo::Mat3d::identity().isSymmetric());
	}

	SECTION("Explicitly symmetric matrix is symmetric")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 2.0, 5.0, 6.0, 3.0, 6.0, 9.0);
		REQUIRE(m.isSymmetric());
	}

	SECTION("Asymmetric matrix is not symmetric")
	{
		ufo::Mat3d m(1.0, 2.0, 0.0, 3.0, 1.0, 0.0, 0.0, 0.0, 1.0);
		REQUIRE(!m.isSymmetric());
	}
}

/**************************************************************************************
|                                                                                     |
|                                   Mat2d tests                                       |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Mat2d] [zeros] Zero factory")
{
	auto m = ufo::Mat2d::zeros();
	REQUIRE(m[0][0] == 0.0);
	REQUIRE(m[0][1] == 0.0);
	REQUIRE(m[1][0] == 0.0);
	REQUIRE(m[1][1] == 0.0);
}

TEST_CASE("[Mat2d] [identity] Identity factory")
{
	auto m = ufo::Mat2d::identity();
	REQUIRE(m[0][0] == 1.0);
	REQUIRE(m[0][1] == 0.0);
	REQUIRE(m[1][0] == 0.0);
	REQUIRE(m[1][1] == 1.0);
}

TEST_CASE("[Mat2d] [ones] Ones factory")
{
	auto m = ufo::Mat2d::ones();
	REQUIRE(m[0][0] == 1.0);
	REQUIRE(m[0][1] == 1.0);
	REQUIRE(m[1][0] == 1.0);
	REQUIRE(m[1][1] == 1.0);
}

TEST_CASE("[Mat2d] [determinant]")
{
	SECTION("Determinant of identity is 1")
	{
		REQUIRE(ufo::determinant(ufo::Mat2d::identity()) == Catch::Approx(1.0));
	}

	SECTION("Determinant of zero matrix is 0")
	{
		REQUIRE(ufo::determinant(ufo::Mat2d::zeros()) == Catch::Approx(0.0));
	}

	SECTION("Determinant of 2x2 matrix (ad - bc)")
	{
		// [[1,2],[3,4]] -> 1*4 - 2*3 = -2
		ufo::Mat2d m(1.0, 2.0, 3.0, 4.0);
		REQUIRE(ufo::determinant(m) == Catch::Approx(-2.0));
	}
}

TEST_CASE("[Mat2d] [inverse]")
{
	SECTION("Inverse of identity is identity")
	{
		auto inv = ufo::inverse(ufo::Mat2d::identity());
		REQUIRE(inv == ufo::Mat2d::identity());
	}

	SECTION("Inverse of 2x2 matrix")
	{
		// [[1,2],[3,4]] -> inv = [[-2,1],[1.5,-0.5]]
		ufo::Mat2d m(1.0, 2.0, 3.0, 4.0);
		auto       inv = ufo::inverse(m);
		REQUIRE(inv[0][0] == Catch::Approx(-2.0));
		REQUIRE(inv[0][1] == Catch::Approx(1.0));
		REQUIRE(inv[1][0] == Catch::Approx(1.5));
		REQUIRE(inv[1][1] == Catch::Approx(-0.5));
	}

	SECTION("M * inverse(M) = identity for 2x2")
	{
		ufo::Mat2d m(1.0, 2.0, 3.0, 4.0);
		auto       id = m * ufo::inverse(m);
		REQUIRE(id[0][0] == Catch::Approx(1.0).margin(1e-10));
		REQUIRE(id[0][1] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(id[1][0] == Catch::Approx(0.0).margin(1e-10));
		REQUIRE(id[1][1] == Catch::Approx(1.0).margin(1e-10));
	}
}

/**************************************************************************************
|                                                                                     |
|                                   Mat4d tests                                       |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Mat4d] [zeros] Zero factory")
{
	auto m = ufo::Mat4d::zeros();
	for (std::size_t r = 0; r < 4; ++r)
		for (std::size_t c = 0; c < 4; ++c) REQUIRE(m[r][c] == 0.0);
}

TEST_CASE("[Mat4d] [identity] Identity factory")
{
	auto m = ufo::Mat4d::identity();
	for (std::size_t r = 0; r < 4; ++r)
		for (std::size_t c = 0; c < 4; ++c) REQUIRE(m[r][c] == (r == c ? 1.0 : 0.0));
}

TEST_CASE("[Mat4d] [ones] Ones factory")
{
	auto m = ufo::Mat4d::ones();
	for (std::size_t r = 0; r < 4; ++r)
		for (std::size_t c = 0; c < 4; ++c) REQUIRE(m[r][c] == 1.0);
}

TEST_CASE("[Mat4d] [determinant]")
{
	SECTION("Determinant of identity is 1")
	{
		REQUIRE(ufo::determinant(ufo::Mat4d::identity()) == Catch::Approx(1.0));
	}

	SECTION("Determinant of zero matrix is 0")
	{
		REQUIRE(ufo::determinant(ufo::Mat4d::zeros()) == Catch::Approx(0.0));
	}

	SECTION("Determinant of diagonal 4x4 is product of diagonal")
	{
		ufo::Mat4d m(2.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0,
		             0.0, 5.0);
		REQUIRE(ufo::determinant(m) == Catch::Approx(120.0));
	}
}

TEST_CASE("[Mat4d] [inverse]")
{
	SECTION("Inverse of identity is identity")
	{
		auto inv = ufo::inverse(ufo::Mat4d::identity());
		REQUIRE(inv == ufo::Mat4d::identity());
	}

	SECTION("M * inverse(M) = identity for 4x4")
	{
		ufo::Mat4d m(2.0, 1.0, 0.0, 0.0, 0.0, 3.0, 1.0, 0.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0,
		             0.0, 5.0);
		auto       id = m * ufo::inverse(m);
		for (std::size_t r = 0; r < 4; ++r)
			for (std::size_t c = 0; c < 4; ++c)
				REQUIRE(id[r][c] == Catch::Approx(r == c ? 1.0 : 0.0).margin(1e-10));
	}
}

/**************************************************************************************
|                                                                                     |
|                                Constructor tests                                    |
|                                                                                     |
**************************************************************************************/

TEST_CASE(
    "[Mat3d] [converting constructor] Converting constructor from different element type")
{
	SECTION("Explicit Mat3f to Mat3d")
	{
		ufo::Mat3f mf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
		ufo::Mat3d md(mf);
		REQUIRE(md[0][0] == Catch::Approx(1.0));
		REQUIRE(md[1][1] == Catch::Approx(5.0));
		REQUIRE(md[2][2] == Catch::Approx(9.0));
	}
}

TEST_CASE("[Mat3d] [cross-size constructor] Cross-size constructor")
{
	SECTION("Mat2d expanded to Mat3d - extra elements are zero")
	{
		ufo::Mat2d small(1.0, 2.0, 3.0, 4.0);
		ufo::Mat3d big(small);
		REQUIRE(big[0][0] == 1.0);
		REQUIRE(big[0][1] == 2.0);
		REQUIRE(big[1][0] == 3.0);
		REQUIRE(big[1][1] == 4.0);
		REQUIRE(big[0][2] == 0.0);
		REQUIRE(big[2][0] == 0.0);
		REQUIRE(big[2][2] == 0.0);
	}

	SECTION("Mat3d truncated to Mat2d - only overlapping elements copied")
	{
		ufo::Mat3d big(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		ufo::Mat2d small(big);
		REQUIRE(small[0][0] == 1.0);
		REQUIRE(small[0][1] == 2.0);
		REQUIRE(small[1][0] == 4.0);
		REQUIRE(small[1][1] == 5.0);
	}
}

/**************************************************************************************
|                                                                                     |
|                            Essential operations tests                               |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Mat3d] [frobenius_norm] Frobenius norm")
{
	SECTION("Frobenius norm of zero matrix is 0")
	{
		REQUIRE(ufo::Mat3d::zeros().frobenius_norm() == Catch::Approx(0.0));
	}

	SECTION("Frobenius norm of identity is sqrt(N)")
	{
		// sqrt(1 + 1 + 1) = sqrt(3)
		REQUIRE(ufo::Mat3d::identity().frobenius_norm() == Catch::Approx(std::sqrt(3.0)));
	}

	SECTION("Frobenius norm of known matrix")
	{
		// [[1,0],[0,2]] -> sqrt(1 + 4) = sqrt(5)
		ufo::Mat2d m(1.0, 0.0, 0.0, 2.0);
		REQUIRE(m.frobenius_norm() == Catch::Approx(std::sqrt(5.0)));
	}
}

TEST_CASE("[Mat3d] [isNearZero] Near-zero check")
{
	SECTION("Zero matrix is near zero") { REQUIRE(ufo::Mat3d::zeros().isNearZero()); }

	SECTION("Identity matrix is not near zero")
	{
		REQUIRE(!ufo::Mat3d::identity().isNearZero());
	}

	SECTION("Very small matrix is near zero with large enough epsilon")
	{
		ufo::Mat3d m(1e-10, 0.0, 0.0, 0.0, 1e-10, 0.0, 0.0, 0.0, 1e-10);
		REQUIRE(m.isNearZero(1e-9));
	}
}

TEST_CASE("[Mat3d] [isUpperTriangular] Upper triangular check")
{
	SECTION("Identity is upper triangular")
	{
		REQUIRE(ufo::Mat3d::identity().isUpperTriangular());
	}

	SECTION("Upper triangular matrix")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 0.0, 9.0);
		REQUIRE(m.isUpperTriangular());
	}

	SECTION("Lower triangular matrix is not upper triangular")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 4.0, 5.0, 0.0, 7.0, 8.0, 9.0);
		REQUIRE(!m.isUpperTriangular());
	}
}

TEST_CASE("[Mat3d] [isLowerTriangular] Lower triangular check")
{
	SECTION("Identity is lower triangular")
	{
		REQUIRE(ufo::Mat3d::identity().isLowerTriangular());
	}

	SECTION("Lower triangular matrix")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 4.0, 5.0, 0.0, 7.0, 8.0, 9.0);
		REQUIRE(m.isLowerTriangular());
	}

	SECTION("Upper triangular matrix is not lower triangular")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 0.0, 9.0);
		REQUIRE(!m.isLowerTriangular());
	}
}

TEST_CASE("[Mat3d] [rank] Matrix rank")
{
	SECTION("Rank of identity is N") { REQUIRE(ufo::Mat3d::identity().rank() == 3); }

	SECTION("Rank of zero matrix is 0") { REQUIRE(ufo::Mat3d::zeros().rank() == 0); }

	SECTION("Rank of singular matrix (linearly dependent rows)")
	{
		// Rows are linearly dependent: row2 = row0 + row1
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		REQUIRE(m.rank() == 2);
	}

	SECTION("Rank of full-rank diagonal matrix")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
		REQUIRE(m.rank() == 3);
	}
}

TEST_CASE("[Mat3d] [safe_inverse] Safe inverse")
{
	SECTION("Safe inverse of identity returns identity")
	{
		auto result = ufo::Mat3d::identity().safe_inverse();
		REQUIRE(result.has_value());
		REQUIRE(result.value() == ufo::Mat3d::identity());
	}

	SECTION("Safe inverse of non-singular matrix returns value")
	{
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0);
		auto       result = m.safe_inverse();
		REQUIRE(result.has_value());
		REQUIRE(result.value()[0][0] == Catch::Approx(1.0));
		REQUIRE(result.value()[1][1] == Catch::Approx(0.5));
		REQUIRE(result.value()[2][2] == Catch::Approx(0.25));
	}

	SECTION("Safe inverse of singular matrix returns nullopt")
	{
		// Singular: rows are linearly dependent
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		auto       result = m.safe_inverse();
		REQUIRE(!result.has_value());
	}
}

TEST_CASE("[Mat3d] [flat_view] Flat span view")
{
	ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

	SECTION("flat_view has correct size") { REQUIRE(m.flat_view().size() == 9); }

	SECTION("flat_view elements are in row-major order")
	{
		auto v = m.flat_view();
		REQUIRE(v[0] == 1.0);
		REQUIRE(v[1] == 2.0);
		REQUIRE(v[2] == 3.0);
		REQUIRE(v[3] == 4.0);
		REQUIRE(v[8] == 9.0);
	}

	SECTION("flat_view is writable")
	{
		m.flat_view()[0] = 99.0;
		REQUIRE(m[0][0] == 99.0);
	}
}

TEST_CASE("[Mat3d] [block] Block extraction")
{
	ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);

	SECTION("2x2 block at (0, 0)")
	{
		auto b = m.block<2>(0, 0);
		REQUIRE(b[0][0] == 1.0);
		REQUIRE(b[0][1] == 2.0);
		REQUIRE(b[1][0] == 4.0);
		REQUIRE(b[1][1] == 5.0);
	}

	SECTION("2x2 block at (1, 1)")
	{
		auto b = m.block<2>(1, 1);
		REQUIRE(b[0][0] == 5.0);
		REQUIRE(b[0][1] == 6.0);
		REQUIRE(b[1][0] == 8.0);
		REQUIRE(b[1][1] == 9.0);
	}
}

TEST_CASE("[Mat3d] [all_of] All-of predicate")
{
	SECTION("All elements of zero matrix are zero")
	{
		REQUIRE(ufo::Mat3d::zeros().all_of([](double e) { return e == 0.0; }));
	}

	SECTION("Not all elements of identity are zero")
	{
		REQUIRE(!ufo::Mat3d::identity().all_of([](double e) { return e == 0.0; }));
	}

	SECTION("All elements of ones matrix are positive")
	{
		REQUIRE(ufo::Mat3d::ones().all_of([](double e) { return e > 0.0; }));
	}
}

TEST_CASE("[Mat3d] [any_of] Any-of predicate")
{
	SECTION("No element of zero matrix is nonzero")
	{
		REQUIRE(!ufo::Mat3d::zeros().any_of([](double e) { return e != 0.0; }));
	}

	SECTION("Identity has at least one nonzero element")
	{
		REQUIRE(ufo::Mat3d::identity().any_of([](double e) { return e != 0.0; }));
	}
}

TEST_CASE("[Mat3d] [transform] Element-wise transform")
{
	SECTION("Doubling each element")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		auto       r = m.transform([](double e) { return e * 2.0; });
		REQUIRE(r[0][0] == 2.0);
		REQUIRE(r[1][1] == 10.0);
		REQUIRE(r[2][2] == 18.0);
	}

	SECTION("Negating each element")
	{
		auto id  = ufo::Mat3d::identity();
		auto neg = id.transform([](double e) { return -e; });
		REQUIRE(neg[0][0] == -1.0);
		REQUIRE(neg[0][1] == 0.0);
	}
}

/**************************************************************************************
|                                                                                     |
|                              Arithmetic operator tests                              |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[Mat3d] [operator/] Scalar division")
{
	ufo::Mat3d m(2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0);

	SECTION("matrix / scalar")
	{
		auto r = m / 2.0;
		REQUIRE(r[0][0] == Catch::Approx(1.0));
		REQUIRE(r[0][1] == Catch::Approx(2.0));
		REQUIRE(r[1][1] == Catch::Approx(5.0));
	}

	SECTION("matrix /= scalar")
	{
		m /= 2.0;
		REQUIRE(m[0][0] == Catch::Approx(1.0));
		REQUIRE(m[1][1] == Catch::Approx(5.0));
		REQUIRE(m[2][2] == Catch::Approx(9.0));
	}

	SECTION("scalar / matrix")
	{
		ufo::Mat3d d(1.0, 2.0, 4.0, 1.0, 5.0, 10.0, 1.0, 8.0, 2.0);
		auto       r = 4.0 / d;
		REQUIRE(r[0][0] == Catch::Approx(4.0));
		REQUIRE(r[0][1] == Catch::Approx(2.0));
		REQUIRE(r[0][2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Mat3d] [scalar-matrix subtraction] Scalar minus matrix")
{
	SECTION("scalar - matrix")
	{
		ufo::Mat3d m(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
		auto       r = 10.0 - m;
		REQUIRE(r[0][0] == Catch::Approx(9.0));
		REQUIRE(r[0][1] == Catch::Approx(8.0));
		REQUIRE(r[1][0] == Catch::Approx(6.0));
		REQUIRE(r[2][2] == Catch::Approx(1.0));
	}
}

TEST_CASE("[Mat3d] [vector * matrix] Row-vector times matrix")
{
	SECTION("(1,1,1) * identity = (1,1,1)")
	{
		ufo::Vec3d v(1.0, 1.0, 1.0);
		auto       r = v * ufo::Mat3d::identity();
		REQUIRE(r[0] == 1.0);
		REQUIRE(r[1] == 1.0);
		REQUIRE(r[2] == 1.0);
	}

	SECTION("(1,1,1) * diag(1,2,3) = (1,2,3)")
	{
		ufo::Vec3d v(1.0, 1.0, 1.0);
		ufo::Mat3d m(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0);
		auto       r = v * m;
		REQUIRE(r[0] == 1.0);
		REQUIRE(r[1] == 2.0);
		REQUIRE(r[2] == 3.0);
	}
}

/**************************************************************************************
|                                                                                     |
|                                Free function tests                                  |
|                                                                                     |
**************************************************************************************/

TEST_CASE("[mat] [zeros<>] Free zeros factory")
{
	SECTION("zeros<3,3,double>() == Mat3d::zeros()")
	{
		REQUIRE(ufo::zeros<3, 3, double>() == ufo::Mat3d::zeros());
	}

	SECTION("zeros<2,4,float>() has correct dimensions and values")
	{
		auto m = ufo::zeros<2, 4, float>();
		for (std::size_t r = 0; r < 2; ++r)
			for (std::size_t c = 0; c < 4; ++c) REQUIRE(m[r][c] == 0.0f);
	}
}

TEST_CASE("[mat] [identity<>] Free identity factory")
{
	SECTION("identity<3,double>() == Mat3d::identity()")
	{
		REQUIRE(ufo::identity<3, double>() == ufo::Mat3d::identity());
	}

	SECTION("identity<2,double>() == Mat2d::identity()")
	{
		REQUIRE(ufo::identity<2, double>() == ufo::Mat2d::identity());
	}
}

TEST_CASE("[mat] [ones<>] Free ones factory")
{
	SECTION("ones<3,3,double>() == Mat3d::ones()")
	{
		REQUIRE(ufo::ones<3, 3, double>() == ufo::Mat3d::ones());
	}

	SECTION("ones<2,3,float>() has correct dimensions and values")
	{
		auto m = ufo::ones<2, 3, float>();
		for (std::size_t r = 0; r < 2; ++r)
			for (std::size_t c = 0; c < 3; ++c) REQUIRE(m[r][c] == 1.0f);
	}
}
