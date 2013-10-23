/*
 * This file is part of libskro.
 *
 * libskro is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * libskro is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTIBILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * libskro. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>

#include <endian.h>

#include <ethercat/ethercat.h>

#include <skro/skro.h>

#define SKRO_READ_BUF(in_ptr, type, value) \
	do { \
		(value) = *((type *)(in_ptr)); \
		in_ptr += sizeof(type); \
	} while (0)
#define SKRO_WRITE_BUF(out_ptr, type, value) \
	do { \
		*((type *)(out_ptr)) = (value); \
		(out_ptr) += sizeof(type); \
	} while (0)

int skro_init(struct skro_ctx_t *ctx, char *if_name) {
	if (!ctx) {
		return SKRO_ERR_INVAL;
	}

	/*
	 * Attempt to establish an EtherCAT connection.
	 */
	if (ecx_init(&ctx->ec_ctx, if_name) < 0) {
		return SKRO_ERR_NO_CONN;
	}

	/*
	 * Attempt to detect and configure all the slaves.
	 */
	ctx->slave_count = ecx_config_init(&ctx->ec_ctx, TRUE);

	if (ctx->slave_count == 0) {
		ecx_close(&ctx->ec_ctx);

		return SKRO_ERR_NO_SLAVES;
	}

	ecx_config_map_group(&ctx->ec_ctx, &ctx->io_map, 0);

	/*
	 * Wait for all slaves to reach SAFE_OP state.
	 */
	ecx_statecheck(&ctx->ec_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

	if (ctx->ec_ctx.slavelist[0].state != EC_STATE_SAFE_OP) {
		/*
		 * TODO: check which slaves haven't reached SAFE_OP state.
		 */
	}

	/* 
	 * Request all slaves to reach OP state.
	 */
	ctx->ec_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;

	ecx_send_processdata(&ctx->ec_ctx);
	ecx_receive_processdata(&ctx->ec_ctx, EC_TIMEOUTRET);
	ec_writestate(0);

	/*
	 * Wait for all slaves to reach OP state.
	 */
	ecx_statecheck(&ctx->ec_ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

	if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
		ecx_close(&ctx->ec_ctx);

		return SKRO_ERR_NOT_READY;
	}

	return SKRO_ERR_OK;
}

void skro_cleanup(struct skro_ctx_t *ctx) {
	if (!ctx) {
		return;
	}

	/*
	 * Request all slaves to reach SAFE_OP state.
	 */
	ctx->ec_ctx.slavelist[0].state = EC_STATE_SAFE_OP;

	ecx_writestate(&ctx->ec_ctx, 0);

	/*
	 * Close the EtherCAT connection.
	 */
	ecx_close(&ctx->ec_ctx);
	free(ctx);
}

int recv_mbx_msg(struct skro_ctx_t *ctx, int slave_id, struct skro_mbx_in_msg_t *in_msg, int timeout) {
	char *in_ptr;

	if (!ctx || slave_id >= ctx->slave_count || !in_msg) {
		return SKRO_ERR_INVAL;
	}

	if (ecx_mbxreceive(&ctx->ec_ctx, slave_id, &ctx->mbx_in_buf, timeout) == 0) {
		return SKRO_ERR_TIMED_OUT;
	}

	in_ptr = (char *)ctx->mbx_in_buf;

	SKRO_READ_BUF(in_ptr, uint8_t, in_msg->reply_addr);
	SKRO_READ_BUF(in_ptr, uint8_t, in_msg->mod_addr);
	SKRO_READ_BUF(in_ptr, uint8_t, in_msg->status);
	SKRO_READ_BUF(in_ptr, uint8_t, in_msg->cmd_id);
	SKRO_READ_BUF(in_ptr, uint32_t, in_msg->value);
	in_msg->value = be32toh(in_msg->value);

	return SKRO_ERR_OK;
}

int send_mbx_msg(struct skro_ctx_t *ctx, int slave_id, struct skro_mbx_out_msg_t *out_msg, int timeout) {
	unsigned char *out_ptr;

	if (!ctx || slave_id >= ctx->slave_count || !out_msg) {
		return SKRO_ERR_INVAL;
	}

	out_ptr = (unsigned char *)ctx->mbx_out_buf;

	SKRO_WRITE_BUF(out_ptr, uint8_t, out_msg->mod_addr);
	SKRO_WRITE_BUF(out_ptr, uint8_t, out_msg->cmd_id);
	SKRO_WRITE_BUF(out_ptr, uint8_t, out_msg->type_id);
	SKRO_WRITE_BUF(out_ptr, uint8_t, out_msg->motor_id);
	SKRO_WRITE_BUF(out_ptr, uint32_t, htobe32(out_msg->value));

	if (ecx_mbxsend(&ctx->ec_ctx, slave_id, &ctx->mbx_out_buf, timeout) == 0) {
		return SKRO_ERR_UNKNOWN;
	}

	return SKRO_ERR_OK;
}

