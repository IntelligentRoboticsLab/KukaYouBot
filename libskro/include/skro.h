#ifndef SKRO_H
#define SKRO_H

struct skro_ctx_t;

struct skro_ctx_t *skro_init(const char *if_name);
void skro_cleanup(struct skro_ctx_t *ctx);

#endif /* SKRO_H */

