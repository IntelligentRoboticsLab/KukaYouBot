/*
 * jsonz.h
 * 
 * Recognition library that uses Google Speech API 
 *
 * Build off of libsprec and libjsonz utilities by Árpád Goretity (H2CO3)
 *
 */

#include <string.h>
#include <stdlib.h>
#include "object.h"

#ifndef JSONZ_JSONZ_H
#define JSONZ_JSONZ_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void *jsonz_parse(const char *str);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* JSONZ_JSONZ_H */

