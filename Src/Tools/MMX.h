/**
 * @file MMX.h
 *
 * The file defines some compiler specific specializations of MMX intrinsics
 *
 * @author Alexander Härtl
 */

#pragma once

#include <mmintrin.h>
#include <mm3dnow.h>
#ifdef MACOSX
#include <xmmintrin.h>
#endif

#define MM_EMPTY _mm_empty

// TODO some of those builtin functions are not available on LLVM
#ifdef LINUX
#define _mm_extract_pi16(A, N) ((int)__builtin_ia32_vec_ext_v4hi((__v4hi)(__m64)(A), (int)(N)))
#define _mm_insert_pi16(A, D, N) ((__m64)__builtin_ia32_vec_set_v4hi((__v4hi)(A), (D), (N)))
#define _mm_max_pi16(A, B) ((__m64)__builtin_ia32_pmaxsw((__v4hi)A, (__v4hi)B))
#define _mm_max_pu8(A, B) ((__m64)__builtin_ia32_pmaxub((__v4hi)A, (__v4hi)B))
#define _mm_min_pi16(A, B) ((__m64)__builtin_ia32_pminsw((__v4hi)A, (__v4hi)B))
#define _mm_min_pu8(A, B) ((__m64)__builtin_ia32_pminub((__v4hi)A, (__v4hi)B))
#define _mm_movemask_pi8(A) (__builtin_ia32_pmovmskb((__v8qi)(A)))
#define _mm_mulhi_pu16(A, B) ((__m64)__builtin_ia32_pmulhuw((__v4hi)(A), (__v4hi)(B)))
#define _mm_shuffle_pi16(A, N) ((__m64)__builtin_ia32_pshufw((__v4hi)(A), (N)))
#define _mm_maskmove_si64(A, B, P) __builtin_ia32_maskmovq((__v8qi)A, (__v8qi)B, P))
#define _mm_avg_pu8(A, B) ((__m64)__builtin_ia32_pavgb((__v8qi)A, (__v8qi)B))
#define _mm_avg_pu16(A, B) ((__m64)__builtin_ia32_pavgw((__v8qi)A, (__v8qi)B))
#define _mm_sad_pu8(A, B) ((__m64)__builtin_ia32_psadbw((__v8qi)A, (__v8qi)B))
#endif
