/**
 * @file Tools/Sha1.cpp
 * @brief SHA-1 Hash key computation implementation
 *
 * 100% free public domain implementation of the SHA-1
 * algorithm by Dominik Reichl <Dominik.Reichl@tiscali.de>
 *
 *
 * === Test Vectors (from FIPS PUB 180-1) ===
 *
 * "abc"
 *  A9993E36 4706816A BA3E2571 7850C26C 9CD0D89D
 *
 * "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
 *  84983E44 1C3BD26E BAAE4AA1 F95129E5 E54670F1
 *
 * A million repetitions of "a"
 *  34AA973C D4C4DAA4 F61EEB2B DBAD2731 6534016F
 */

#include <cstdlib>
#include <cstdio> // Needed for file access
#include <cstring> // Needed for strcat and strcpy

#include "Sha1.h"

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN
#endif

/**
 * Size of an SHA-1 key in byte
 */
#define SHA1_LENGTH 20


// Rotate x bits to the left
#define ROL32(value, bits) (((value)<<(bits))|((value)>>(32-(bits))))


#ifdef LITTLE_ENDIAN
#define SHABLK0(i) (block->l[i] = (ROL32(block->l[i],24) & 0xFF00FF00) \
                                  | (ROL32(block->l[i],8) & 0x00FF00FF))
#else
#define SHABLK0(i) (block->l[i])
#endif

#define SHABLK(i) (block->l[i&15] = ROL32(block->l[(i+13)&15] ^ block->l[(i+8)&15] \
                                    ^ block->l[(i+2)&15] ^ block->l[i&15],1))

// SHA-1 rounds
#define R0(v,w,x,y,z,i) { z+=((w&(x^y))^y)+SHABLK0(i)+0x5A827999+ROL32(v,5); w=ROL32(w,30); }
#define R1(v,w,x,y,z,i) { z+=((w&(x^y))^y)+SHABLK(i)+0x5A827999+ROL32(v,5); w=ROL32(w,30); }
#define R2(v,w,x,y,z,i) { z+=(w^x^y)+SHABLK(i)+0x6ED9EBA1+ROL32(v,5); w=ROL32(w,30); }
#define R3(v,w,x,y,z,i) { z+=(((w|x)&y)|(w&x))+SHABLK(i)+0x8F1BBCDC+ROL32(v,5); w=ROL32(w,30); }
#define R4(v,w,x,y,z,i) { z+=(w^x^y)+SHABLK(i)+0xCA62C1D6+ROL32(v,5); w=ROL32(w,30); }


Sha1::Sha1()
{
  reset();
}

Sha1::~Sha1()
{
}


void Sha1::reset()
{
  // SHA1 initialization constants
  m_state[ 0 ] = 0x67452301;
  m_state[ 1 ] = 0xEFCDAB89;
  m_state[ 2 ] = 0x98BADCFE;
  m_state[ 3 ] = 0x10325476;
  m_state[ 4 ] = 0xC3D2E1F0;

  m_count[ 0 ] = 0;
  m_count[ 1 ] = 0;
}

void Sha1::transform(unsigned state[ 5 ], const unsigned char buffer[ 64 ])
{
  unsigned a = 0, b = 0, c = 0, d = 0, e = 0;

  SHA1_WORKSPACE_BLOCK* block;
  // static unsigned char workspace[64];
  block = (SHA1_WORKSPACE_BLOCK*) workspace;
  memcpy(block, buffer, 64);

  // Copy state[] to working vars
  a = state[ 0 ];
  b = state[ 1 ];
  c = state[ 2 ];
  d = state[ 3 ];
  e = state[ 4 ];

  // 4 rounds of 20 operations each. Loop unrolled.
  R0(a, b, c, d, e, 0);
  R0(e, a, b, c, d, 1);
  R0(d, e, a, b, c, 2);
  R0(c, d, e, a, b, 3);
  R0(b, c, d, e, a, 4);
  R0(a, b, c, d, e, 5);
  R0(e, a, b, c, d, 6);
  R0(d, e, a, b, c, 7);
  R0(c, d, e, a, b, 8);
  R0(b, c, d, e, a, 9);
  R0(a, b, c, d, e, 10);
  R0(e, a, b, c, d, 11);
  R0(d, e, a, b, c, 12);
  R0(c, d, e, a, b, 13);
  R0(b, c, d, e, a, 14);
  R0(a, b, c, d, e, 15);
  R1(e, a, b, c, d, 16);
  R1(d, e, a, b, c, 17);
  R1(c, d, e, a, b, 18);
  R1(b, c, d, e, a, 19);
  R2(a, b, c, d, e, 20);
  R2(e, a, b, c, d, 21);
  R2(d, e, a, b, c, 22);
  R2(c, d, e, a, b, 23);
  R2(b, c, d, e, a, 24);
  R2(a, b, c, d, e, 25);
  R2(e, a, b, c, d, 26);
  R2(d, e, a, b, c, 27);
  R2(c, d, e, a, b, 28);
  R2(b, c, d, e, a, 29);
  R2(a, b, c, d, e, 30);
  R2(e, a, b, c, d, 31);
  R2(d, e, a, b, c, 32);
  R2(c, d, e, a, b, 33);
  R2(b, c, d, e, a, 34);
  R2(a, b, c, d, e, 35);
  R2(e, a, b, c, d, 36);
  R2(d, e, a, b, c, 37);
  R2(c, d, e, a, b, 38);
  R2(b, c, d, e, a, 39);
  R3(a, b, c, d, e, 40);
  R3(e, a, b, c, d, 41);
  R3(d, e, a, b, c, 42);
  R3(c, d, e, a, b, 43);
  R3(b, c, d, e, a, 44);
  R3(a, b, c, d, e, 45);
  R3(e, a, b, c, d, 46);
  R3(d, e, a, b, c, 47);
  R3(c, d, e, a, b, 48);
  R3(b, c, d, e, a, 49);
  R3(a, b, c, d, e, 50);
  R3(e, a, b, c, d, 51);
  R3(d, e, a, b, c, 52);
  R3(c, d, e, a, b, 53);
  R3(b, c, d, e, a, 54);
  R3(a, b, c, d, e, 55);
  R3(e, a, b, c, d, 56);
  R3(d, e, a, b, c, 57);
  R3(c, d, e, a, b, 58);
  R3(b, c, d, e, a, 59);
  R4(a, b, c, d, e, 60);
  R4(e, a, b, c, d, 61);
  R4(d, e, a, b, c, 62);
  R4(c, d, e, a, b, 63);
  R4(b, c, d, e, a, 64);
  R4(a, b, c, d, e, 65);
  R4(e, a, b, c, d, 66);
  R4(d, e, a, b, c, 67);
  R4(c, d, e, a, b, 68);
  R4(b, c, d, e, a, 69);
  R4(a, b, c, d, e, 70);
  R4(e, a, b, c, d, 71);
  R4(d, e, a, b, c, 72);
  R4(c, d, e, a, b, 73);
  R4(b, c, d, e, a, 74);
  R4(a, b, c, d, e, 75);
  R4(e, a, b, c, d, 76);
  R4(d, e, a, b, c, 77);
  R4(c, d, e, a, b, 78);
  R4(b, c, d, e, a, 79);

  // Add the working vars back into state[]
  state[ 0 ] += a;
  state[ 1 ] += b;
  state[ 2 ] += c;
  state[ 3 ] += d;
  state[ 4 ] += e;

  // Wipe variables
  a = 0;
  b = 0;
  c = 0;
  d = 0;
  e = 0;
}

// Use this function to hash in binary data and strings
void Sha1::update(const unsigned char* data, unsigned int len)
{
  unsigned i = 0, j = 0;

  j = (m_count[ 0 ] >> 3) & 63;

  if((m_count[ 0 ] += len << 3) < (len << 3))
    m_count[ 1 ] ++;

  m_count[ 1 ] += (len >> 29);

  if((j + len) > 63)
  {
    memcpy(&m_buffer[ j ], data, (i = 64 - j));
    transform(m_state, m_buffer);

    for(; i + 63 < len; i += 64)
    {
      transform(m_state, &data[ i ]);
    }

    j = 0;
  }

  else
    i = 0;

  memcpy(&m_buffer[ j ], &data[ i ], len - i);
}

void Sha1::final()
{
  unsigned i = 0;
  unsigned char finalcount[ 8 ] =
  {
    0, 0, 0, 0, 0, 0, 0, 0
  };

  for(i = 0; i < 8; i++)
    finalcount[ i ] = (unsigned char)((m_count[(i >= 4 ? 0 : 1) ]
                                       >> ((3 - (i & 3)) * 8)) & 255);         // Endian independent

  update((unsigned char*) "\200", 1);

  while((m_count[ 0 ] & 504) != 448)
    update((unsigned char*) "\0", 1);

  update(finalcount, 8);   // Cause a SHA1Transform()

  for(i = 0; i < 20; i++)
  {
    m_digest[ i ] = (unsigned char)((m_state[ i >> 2 ] >> ((3 - (i & 3)) * 8)) & 255);
  }

  // Wipe variables for security reasons
  i = 0;

  memset(m_buffer, 0, 64);

  memset(m_state, 0, 20);

  memset(m_count, 0, 8);

  memset(finalcount, 0, 8);

  transform(m_state, m_buffer);
}

// Get the raw message digest
void Sha1::getHash(unsigned char* uDest) const
{
  memcpy(uDest, m_digest, 20);
}

// Get the raw message digest
// Added by Kevin to be quicker
unsigned char* Sha1::getHash(void) const
{
  return (unsigned char*) m_digest;
}

void Sha1::sha1(const unsigned char* data, unsigned int len, unsigned char* result)
{
  Sha1 sha1;
  sha1.update(data, len);
  sha1.final();
  sha1.getHash(result);
}
