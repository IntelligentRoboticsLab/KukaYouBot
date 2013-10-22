/*
 * This file is part of libskro.
 *
 * libskro is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * libskro is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTIBILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libskro. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <string.h>

#include <ethercat/ethercat.h>

#include "skro.h"

struct skro_ctx_t *skro_init(char *if_name) {
	struct skro_ctx_t *ctx;

	/*
	 * Create the context object that is used within libskro.
	 */
	ctx = calloc(1, sizeof(struct skro_ctx_t));

	if (!ctx) {
		goto cleanup0;
	}

	/*
	 * Attempt to initiate libsoem, and bind a socket to the interface that
	 * has been specified.
	 */
	if (ecx_init(&ctx->ec_ctx, if_name) < 0) {
		goto cleanup1;
	}

	/*
	 * Attempt to detect and configure all the slaves.
	 */
	ctx->slave_count = ecx_config_init(&ctx->ec_ctx, TRUE);

	if (ctx->slave_count == 0) {
		goto cleanup2;
	}

	ecx_config_map_group(&ctx->ec_ctx, &ctx->io_map, 0);

	return ctx;

cleanup2:
	ecx_close(&ctx->ec_ctx);
cleanup1:
	free(ctx);
cleanup0:
	return NULL;
}

void skro_cleanup(struct skro_ctx_t *ctx) {
	if (!ctx) {
		return;
	}

	ecx_close(&ctx->ec_ctx);
	free(ctx);
}

