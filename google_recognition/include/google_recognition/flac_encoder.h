/*
 * flac_encoder.h
 * 
 * Recognition library that uses Google Speech API 
 *
 * Build off of libsprec and libjsonz utilities by Árpád Goretity (H2CO3)
 *
 */

#ifndef __SPREC_FLAC_ENCODER_H__
#define __SPREC_FLAC_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/*
 * Converts a WAV PCM file at the path `wavfile'
 * to a FLAC file with the same sample rate,
 * channel number and bit depth. Writes the result
 * to the file at path `flacfile'.
 * Returns 0 on success, non-0 on error.
 */
int sprec_flac_encode(const char *wavfile, const char *flacfile);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !__SPREC_FLAC_ENCODER_H__ */

