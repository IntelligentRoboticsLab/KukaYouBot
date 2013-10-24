/*
 * This file is part of libtmcl.
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

#include <tmcl/tmcl.h>

#define TMCL_ENC_BUF(out_ptr, type, value) \
	do { \
		*((type *)(out_ptr)) = (value); \
		(out_ptr) += sizeof(type); \
	} while (0)
#define TMCL_DEC_BUF(in_ptr, type, value) \
	do { \
		(value) = *((type *)(in_ptr)); \
		(in_ptr) += sizeof(type); \
	} while (0)

int tmcl_init(struct tmcl_ctx_t *ctx, unsigned int flags) {
	if (!ctx) {
		return -1;
	}

	memset(ctx, 0, sizeof(struct tmcl_ctx_t));

	/*
	 * Compute the required sizes for the command and the command reply
	 * buffers.
	 */
	ctx->req_cmd_size = 3 * sizeof(uint8_t) + sizeof(uint32_t);
	ctx->req_reply_size = 3 * sizeof(uint8_t) + sizeof(uint32_t);

	if (flags & TMCL_HAS_ADDR) {
		ctx->flags |= TMCL_HAS_ADDR;
		ctx->req_cmd_size += sizeof(uint8_t);
		ctx->req_reply_size += sizeof(uint8_t);
	}

	if (flags & TMCL_HAS_CHKSUM) {
		ctx->flags |= TMCL_HAS_CHKSUM;
		ctx->req_cmd_size += sizeof(uint8_t);
		ctx->req_reply_size += sizeof(uint8_t);
	}

	return 0;
}

void tmcl_quit(struct tmcl_ctx_t *ctx) {
	if (!ctx) {
		return;
	}

	memset(ctx, 0, sizeof(struct tmcl_ctx_t));
}

unsigned int tmcl_chksum(char *buf, size_t size) {
	unsigned int res = 0;
	size_t i;

	for (i = 0; i < size; ++i) {
		res += *buf++;
	}

	return res;
}

int tmcl_enc_cmd(struct tmcl_ctx_t *ctx, char *buf, size_t size, struct tmcl_cmd_t *cmd) {
	char *out_ptr = buf;

	if (!ctx || !buf || !cmd || size < ctx->req_cmd_size) {
		return 0;
	}

	if (ctx->flags & TMCL_HAS_ADDR) {
		TMCL_ENC_BUF(out_ptr, uint8_t, cmd->mod_addr);
	}

	TMCL_ENC_BUF(out_ptr, uint8_t, cmd->cmd_id);
	TMCL_ENC_BUF(out_ptr, uint8_t, cmd->type_id);
	TMCL_ENC_BUF(out_ptr, uint8_t, cmd->motor_id);
	TMCL_ENC_BUF(out_ptr, uint32_t, htobe32(cmd->value));

	if (ctx->flags & TMCL_HAS_CHKSUM) {
		cmd->chksum = tmcl_chksum(buf, out_ptr - buf);
		TMCL_ENC_BUF(out_ptr, uint8_t, cmd->chksum);
	}

	return (out_ptr - buf);
}

int tmcl_dec_cmd(struct tmcl_ctx_t *ctx, struct tmcl_cmd_t *cmd, char *buf, size_t size) {
	char *in_ptr = buf;

	if (!ctx || !buf || !cmd || size < ctx->req_cmd_size) {
		return 0;
	}

	if (ctx->flags & TMCL_HAS_ADDR) {
		TMCL_DEC_BUF(in_ptr, uint8_t, cmd->mod_addr);
	}

	TMCL_DEC_BUF(in_ptr, uint8_t, cmd->cmd_id);
	TMCL_DEC_BUF(in_ptr, uint8_t, cmd->type_id);
	TMCL_DEC_BUF(in_ptr, uint8_t, cmd->motor_id);
	TMCL_DEC_BUF(in_ptr, uint32_t, cmd->value);
	cmd->value = be32toh(cmd->value);

	if (ctx->flags & TMCL_HAS_CHKSUM) {
		TMCL_DEC_BUF(in_ptr, uint8_t, cmd->chksum);
	}

	return (in_ptr - buf);
}

int tmcl_enc_cmd_reply(struct tmcl_ctx_t *ctx, char *buf, size_t size, struct tmcl_cmd_reply_t *reply) {
	char *out_ptr = buf;

	if (!ctx || !buf || !reply || size < ctx->req_reply_size) {
		return 0;
	}

	if (ctx->flags & TMCL_HAS_ADDR) {
		TMCL_ENC_BUF(out_ptr, uint8_t, reply->reply_addr);
	}

	TMCL_ENC_BUF(out_ptr, uint8_t, reply->mod_addr);
	TMCL_ENC_BUF(out_ptr, uint8_t, reply->cmd_id);
	TMCL_ENC_BUF(out_ptr, uint8_t, reply->type_id);
	TMCL_ENC_BUF(out_ptr, uint8_t, reply->motor_id);
	TMCL_ENC_BUF(out_ptr, uint32_t, htobe32(reply->value));

	if (ctx->flags & TMCL_HAS_CHKSUM) {
		reply->chksum = tmcl_chksum(buf, out_ptr - buf);
		TMCL_ENC_BUF(out_ptr, uint8_t, reply->chksum);
	}

	return (out_ptr - buf);
}

int tmcl_dec_cmd_reply(struct tmcl_ctx_t *ctx, struct tmcl_cmd_reply_t *reply, char *buf, size_t size) {
	char *in_ptr = buf;

	if (!ctx || !buf || !reply || size < ctx->req_reply_size) {
		return 0;
	}

	if (ctx->flags & TMCL_HAS_ADDR) {
		TMCL_DEC_BUF(in_ptr, uint8_t, reply->reply_addr);
	}

	TMCL_DEC_BUF(in_ptr, uint8_t, reply->mod_addr);
	TMCL_DEC_BUF(in_ptr, uint8_t, reply->cmd_id);
	TMCL_DEC_BUF(in_ptr, uint8_t, reply->type_id);
	TMCL_DEC_BUF(in_ptr, uint8_t, reply->motor_id);
	TMCL_DEC_BUF(in_ptr, uint32_t, reply->value);
	reply->value = be32toh(reply->value);

	if (ctx->flags & TMCL_HAS_CHKSUM) {
		TMCL_DEC_BUF(in_ptr, uint8_t, reply->chksum);
	}

	return (in_ptr - buf);
}

