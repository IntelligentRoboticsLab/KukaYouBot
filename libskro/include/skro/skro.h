#ifndef SKRO_H
#define SKRO_H

#include <skro/error.h>

#define SKRO_IO_MAP_SIZE 4096

struct skro_ctx_t {
	char io_map[SKRO_IO_MAP_SIZE];
	ecx_contextt ec_ctx;
	int slave_count;
};

int skro_init(struct skro_ctx_t *ctx, char *if_name);
void skro_cleanup(struct skro_ctx_t *ctx);

#endif /* SKRO_H */

