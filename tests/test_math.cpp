#include "../src/math/Vec2.h"
#include <iostream>
#include <cassert>
#include <cmath>

void TestVec2Basics() {
    std::cout << "[Test Math: Vec2 Basics] Testing constructors, additions, scalar multiplications...\n";
    Velox::Vec2 a(3.0f, 4.0f);
    Velox::Vec2 b(1.0f, 2.0f);

    // Addition
    Velox::Vec2 add = a + b;
    assert(std::abs(add.x - 4.0f) < 1e-5f);
    assert(std::abs(add.y - 6.0f) < 1e-5f);

    // Subtraction
    Velox::Vec2 sub = a - b;
    assert(std::abs(sub.x - 2.0f) < 1e-5f);
    assert(std::abs(sub.y - 2.0f) < 1e-5f);

    // Scalar Multiplication & Division
    Velox::Vec2 mul = a * 2.0f;
    assert(std::abs(mul.x - 6.0f) < 1e-5f);
    assert(std::abs(mul.y - 8.0f) < 1e-5f);

    Velox::Vec2 div = a / 2.0f;
    assert(std::abs(div.x - 1.5f) < 1e-5f);
    assert(std::abs(div.y - 2.0f) < 1e-5f);

    // Compound assignment
    Velox::Vec2 c = a;
    c += b;
    assert(std::abs(c.x - 4.0f) < 1e-5f && std::abs(c.y - 6.0f) < 1e-5f);
    c -= b;
    assert(std::abs(c.x - 3.0f) < 1e-5f && std::abs(c.y - 4.0f) < 1e-5f);
    c *= 3.0f;
    assert(std::abs(c.x - 9.0f) < 1e-5f && std::abs(c.y - 12.0f) < 1e-5f);

    std::cout << "  -> PASSED: Vec2 arithmetic operations verified.\n";
}

void TestVec2Geometric() {
    std::cout << "[Test Math: Vec2 Geometric] Testing Dot product, Magnitude, Normalization, Rotation...\n";
    Velox::Vec2 v(3.0f, 4.0f);

    // Dot product
    Velox::Vec2 u(2.0f, -1.0f);
    Velox::Real dot = v.Dot(u); // 3*2 + 4*(-1) = 2
    assert(std::abs(dot - 2.0f) < 1e-5f);

    // Magnitude & MagnitudeSqr
    assert(std::abs(v.MagnitudeSqr() - 25.0f) < 1e-5f);
    assert(std::abs(v.Magnitude() - 5.0f) < 1e-5f);

    // Normalization
    Velox::Vec2 norm = v.Normalized();
    assert(std::abs(norm.Magnitude() - 1.0f) < 1e-5f);
    assert(std::abs(norm.x - 0.6f) < 1e-5f);
    assert(std::abs(norm.y - 0.8f) < 1e-5f);

    // Zero vector normalization safety
    Velox::Vec2 zero(0.0f, 0.0f);
    Velox::Vec2 normZero = zero.Normalized();
    assert(normZero.x == 0.0f && normZero.y == 0.0f);

    // Rotation (90 degrees CCW = pi/2)
    Velox::Vec2 right(1.0f, 0.0f);
    Velox::Vec2 rotated = right.Rotate(1.57079632679f); // 90 deg -> (0, 1)
    assert(std::abs(rotated.x - 0.0f) < 1e-4f);
    assert(std::abs(rotated.y - 1.0f) < 1e-4f);

    std::cout << "  -> PASSED: Vec2 geometric functions verified.\n";
}

int main() {
    std::cout << "=== Running Velox Math Unit Tests ===\n";
    TestVec2Basics();
    TestVec2Geometric();
    std::cout << "=== All Math Unit Tests Passed Successfully! ===\n";
    return 0;
}
