#ifndef SKRO_H
#define SKRO_H

#include <skro/error.h>

#define SKRO_IO_MAP_SIZE 4096

struct skro_ctx_t {
	char io_map[SKRO_IO_MAP_SIZE];
	ec_mbxbuft mbx_in_buf, mbx_out_buf;
	ecx_contextt ec_ctx;
	int slave_count;
};

struct skro_mbx_in_msg_t {
	uint32_t value;
	uint8_t reply_addr, mod_addr, status, cmd_id;
};

struct skro_mbx_out_msg_t {
	uint32_t value;
	uint8_t mod_addr, cmd_id, type_id, motor_id;
};

int skro_init(struct skro_ctx_t *ctx, char *if_name);
void skro_cleanup(struct skro_ctx_t *ctx);

#endif /* SKRO_H */

