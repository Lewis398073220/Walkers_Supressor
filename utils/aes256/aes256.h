#ifndef uint8_t
#define uint8_t  unsigned char
#endif
#ifndef size_t
#include <stddef.h>
#endif

#define aes256_decrypt_ctr  aes256_encrypt_ctr

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t nonce[4];
	uint8_t iv[8];
	uint8_t ctr[4];
} rfc3686_blk;

typedef struct {
	uint8_t key[32];
	uint8_t enckey[32];
	rfc3686_blk blk;
} aes256_context;


void aes256_init(aes256_context *ctx, uint8_t *key);
void aes256_done(aes256_context *ctx);
void aes256_encrypt_ecb(aes256_context *ctx, uint8_t *buf);

void aes256_setCtrBlk(aes256_context *ctx, rfc3686_blk *blk);
void aes256_encrypt_ctr( uint8_t *buf, size_t sz);

void aes256_deencry_init( uint8_t *key ) ;
void aes256_encry_init(uint8_t *key);

#ifdef __cplusplus
}
#endif
