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

#ifndef TMCL_H
#define TMCL_H

#include <tmcl/tmcl_defs.h>

enum tmcl_flags_t {
	TMCL_HAS_ADDR,
	TMCL_HAS_CHKSUM,
};

struct tmcl_ctx_t {
	size_t req_cmd_size, req_reply_size;
	unsigned int flags;
};

int tmcl_init(struct tmcl_ctx_t *ctx, unsigned int flags);
void tmcl_quit(struct tmcl_ctx_t *ctx);
unsigned int tmcl_chksum(char *buf, size_t size);
int tmcl_enc_cmd(struct tmcl_ctx_t *ctx, char *buf, size_t size, struct tmcl_cmd_t *cmd);
int tmcl_dec_cmd(struct tmcl_ctx_t *ctx, struct tmcl_cmd_t *cmd, char *buf, size_t size);
int tmcl_enc_cmd_reply(struct tmcl_ctx_t *ctx, char *buf, size_t size, struct tmcl_cmd_reply_t *reply);
int tmcl_dec_cmd_reply(struct tmcl_ctx_t *ctx, struct tmcl_cmd_reply_t *reply, char *buf, size_t size);

#endif /* TMCL_H */

