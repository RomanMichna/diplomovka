/**
* @file Tools/Sha1.h
* @brief SHA-1 computation class
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

#pragma once

/**
* Computes SHA-1 hashing class
*/
class Sha1
{
public:

  /**
  * Computes the SHA-1 of a data block.
  * @param data The source of the data block
  * @param len The length of the data block
  * @param result 20 byte output buffer to write the result to
  */
  static void sha1(const unsigned char* data, unsigned int len, unsigned char* result);

  /**
   * Default Constructor
   */
  Sha1();

  /**
   * Destructor
   */
  virtual ~Sha1();

  /**
   * Reset the internal state of the SHA-1 computation
   * to compute a new SHA-1 hash key
   */
  void reset();

  /**
   * Update the hash value from a byte buffer
   * @param data the byte buffer
   * @param len the number of byte available in the byte buffer
   */
  void update(const unsigned char* data, unsigned int len);

  /**
   * Finalize hash and report
   */
  void final();

  /**
   * Retrieve the Hash in a previously allocated array
   * @param uDest the destination array
   */
  void getHash(unsigned char* uDest) const;

  /**
   * Get a pointer to the hash key data.
   */
  unsigned char* getHash(void) const;

private:
  typedef union
  {
    unsigned char c[ 64 ];
    unsigned l[ 16 ];
  } SHA1_WORKSPACE_BLOCK;

  unsigned m_state[ 5 ];
  unsigned m_count[ 2 ];
  unsigned char m_buffer[ 64 ];
  unsigned char m_digest[ 20 ];

  unsigned char workspace[ 64 ];

  /**
   * Private SHA-1 transformation
   */
  void transform(unsigned state[ 5 ], const unsigned char buffer[ 64 ]);
};
