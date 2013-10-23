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

#include <ethercat/ethercat.h>

#include <skro/skro.h>

int skro_init(struct skro_ctx_t *ctx, char *if_name) {
	if (!ctx) {
		return SKRO_ERR_INVAL;
	}

	/*
	 * Attempt to initiate libsoem, and bind a socket to the interface that
	 * has been specified.
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

	ecx_close(&ctx->ec_ctx);
	free(ctx);
}

