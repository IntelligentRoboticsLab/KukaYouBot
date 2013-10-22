#ifndef _SKRO_H
#define _SKRO_H

#define SKRO_IO_MAP_SIZE 4096

struct skro_ctx_t {
	char io_map[SKRO_IO_MAP_SIZE];
	ecx_contextt ec_ctx;
	int slave_count;
};

#endif /* _SKRO_H */

