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
/*vec2f randomDirection(const vec2i ipoint)
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
}*/

constexpr float PERLIN_FACTOR_1D = 256.0f;        // 8-bits
constexpr float PERLIN_FACTOR_2D = 65536.0f;      // 16-bits
constexpr float PERLIN_FACTOR_3D = 16777216.0f;   // 24-bits
constexpr float PERLIN_FACTOR_4D = 4294967296.0f; // 32-bits

float grad1D( uint32_t iValue, float weight )
{
    return ( PERMUTATIONS[iValue & 0xFFu] & 1u ) ? weight : -weight;
}

float grad2D( vec2i iPoint, vec2f weight )
{
    uint8_t a = PERMUTATIONS[iPoint.u_x & 0xFFu];
    uint8_t b = PERMUTATIONS[iPoint.u_y & 0xFFu];

    float angle = Math::TWO_PI<float> * static_cast<float>( a | ( b << 8 ) ) / PERLIN_FACTOR_2D;
    vec2f direction = vec2f{ cosf( angle ), sinf( angle ) };

    return Math::dot( direction, weight );
}

float grad3D( const vec3i& iPoint, const vec3f& weight )
{
    uint8_t a = PERMUTATIONS[iPoint.u_x & 0xFFu];
    uint8_t b = PERMUTATIONS[iPoint.u_y & 0xFFu];
    uint8_t c = PERMUTATIONS[iPoint.u_z & 0xFFu];

    uint32_t rx = ( a & 0x0F ) | ( ( b & 0x0F ) << 4 ) | ( ( c & 0x0F ) << 8 ); // 12-bits
    uint32_t ry = ( ( a & 0xF0 ) >> 4 ) | ( b & 0xF0 ) | ( ( c & 0xF0 ) << 4 ); // 12-bits
    float angleA = Math::TWO_PI<float> * static_cast<float>( rx ) / PERLIN_FACTOR_3D;
    float angleB = ( Math::PI<float> * static_cast<float>( ry ) - Math::HALF_PI<float> ) / PERLIN_FACTOR_3D;
    vec3f direction = { cosf( angleA ) * cosf( angleB ), sinf( angleA ) * cosf( angleB ), sinf( angleB ) };
    
    return Math::dot_3D( direction, weight );
}

float Perlin::perlin1D( float value )
{
    uint32_t ix = static_cast<uint32_t>( value );
    float weight = value - ix;

    float n0 = grad1D( ix, weight );
    float n1 = grad1D( ix + 1u, weight );

    return Math::lerp( n0, n1, weight );
}

float Perlin::perlin2D( const vec2f point )
{
    vec2i ip = (vec2i)point;
    vec2f weight = point - Math::floor( point );

    // Calculate the noise at each of the 4 corners of the region
    vec2f n0 = { grad2D( ip,                 weight ), grad2D( ip + vec2i{ 0, 1 }, weight ) };
    vec2f n1 = { grad2D( ip + vec2i{ 1, 0 }, weight ), grad2D( ip + vec2i{ 1, 1 }, weight ) };
    
    // Interpolate the result between the 4 corners
    vec2f result = Math::lerp( n0, n1, weight.x );
    return Math::lerp( result.x, result.y, weight.y );
}

float Perlin::perlin3D( const vec3f& point )
{
    vec3i ip = (vec3i)point;
    vec3f weight = point - Math::floor( point );

    // Calculate the noise at each of the 4 corners of the region
    vec4f fz0 = { grad3D( ip                   , weight ),
                  grad3D( ip + vec3i{ 1, 0, 0 }, weight ),
                  grad3D( ip + vec3i{ 0, 1, 0 }, weight ),
                  grad3D( ip + vec3i{ 1, 1, 0 }, weight ) };
    vec4f fz1 = { grad3D( ip + vec3i{ 0, 0, 1 }, weight ),
                  grad3D( ip + vec3i{ 1, 0, 1 }, weight ),
                  grad3D( ip + vec3i{ 0, 1, 1 }, weight ),
                  grad3D( ip + vec3i{ 1, 1, 1 }, weight ) };

    vec4f fz = Math::lerp( fz0, fz1, weight.z );
    vec2f fy = Math::lerp( fz.front, fz.back, weight.y );
    return Math::lerp( fy.x, fy.y, weight.x );
}