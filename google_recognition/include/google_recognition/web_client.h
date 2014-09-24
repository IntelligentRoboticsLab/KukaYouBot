/*
 * web_client.h
 * libsprec
 * 
 * Created by Árpád Goretity (H2CO3)
 * on Tue 17/04/2012
 */

#ifndef __SPREC_WEB_CLIENT_H__
#define __SPREC_WEB_CLIENT_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

struct sprec_server_response {
	char *data;
	int length;
};

/*
 * Sends the FLAC-encoded audio data.
 * Returns a struct server_response pointer,
 * in which the API's JSON response is present.
 * Should be freed with sprec_free_response().
 * Returns NULL on error.
 */
struct sprec_server_response *sprec_send_audio_data(void *data, int length, const char *language, uint32_t sample_rate);

void sprec_free_response(struct sprec_server_response *resp);

/*
 * Duplicates the actually useful (i. e. text) part of the JSON
 * data received. Must be free()'d after use.
 * Returns NULL on error.
 */
char *sprec_get_text_from_json(const char *json);

/*
 * Gets the confidence of the recognition algorithm.
 * This may vary from 0.0 (not sure at all)
 * to 1.0 (absolutely sure).
 */
double sprec_get_confidence_from_json(const char *json);

/*
 * Reads an entire file into memory. *buf should be free()'d after use.
 * Returns 0 on success, non-0 on error
 */
int sprec_get_file_contents(const char *file, void **buf, size_t *size);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !__SPREC_WEB_CLIENT_H__ */

