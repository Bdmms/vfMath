#include "../include/vfMath/Perlin.hpp"
#include "../include/vfMath/VectorMath.hpp"

// Random Sequence
unsigned char PERMUTATIONS[] = { 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140, 36,
                      103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0,
                      26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56,
                      87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166,
                      77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55,
                      46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76, 132,
                      187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109,
                      198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126,
                      255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183,
                      170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43,
                      172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112,
                      104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162,
                      241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106,
                      157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205,
                      93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180 };

// Generates a random direction vector at the point
vec2f randomDirection( const vec2i ipoint )
{
    constexpr uint32_t HALF_BIT_LENGTH = 16;
    constexpr float DIVISOR = float( Math::MAX<int> + 1u );

    uint32_t a = ipoint.u_x;
    uint32_t b = ipoint.u_y;

    a *= 0xC3C04403u;
    b ^= ( a << HALF_BIT_LENGTH ) | ( a >> HALF_BIT_LENGTH );
    b *= 0x71EF7DCDu;
    a ^= ( b << HALF_BIT_LENGTH ) | ( b >> HALF_BIT_LENGTH );
    a *= 0x7A1865FDu;
    float angle = Math::PI<float> * ( a / DIVISOR );

    return vec2f{ sinf( angle ), cosf( angle ) };
}

// Calculates the product between the point and the base point direction
float grad2D( const vec2i basePoint, const vec2f point )
{
    return Math::dot( randomDirection( basePoint ), point - (vec2f)basePoint );
}

float Perlin::perlin2D( const vec2f point )
{
    vec2i ip = (vec2i)point;
    vec2f weight = point - (vec2f)ip;

    // Calculate the noise at each of the 4 corners of the region
    vec2f n0 = { grad2D( ip,                 point ), grad2D( ip + vec2i{ 0, 1 }, point ) };
    vec2f n1 = { grad2D( ip + vec2i{ 1, 0 }, point ), grad2D( ip + vec2i{ 1, 1 }, point ) };
    
    // Interpolate the result between the 4 corners
    vec2f result = Math::lerp( n0, n1, weight.x );
    return Math::lerp( result.x, result.y, weight.y );
}