/** Logging routines that depend on libconfig.
 *
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 *********************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "config.h"
#include "log.h"
#include "log_config.h"
#include "utils.h"

int log_parse(struct log *l, config_setting_t *cfg)
{
	const char *facilities;
	
	if (!config_setting_is_group(cfg))
		cerror(cfg, "Setting 'log' must be a group.");

	config_setting_lookup_int(cfg, "level", &l->level);
	config_setting_lookup_string(cfg, "file", &l->path);

	if (config_setting_lookup_string(cfg, "facilities", &facilities))
		log_set_facility_expression(l, facilities);

	l->state = STATE_PARSED;

	return 0;
}

void cerror(config_setting_t *cfg, const char *fmt, ...)
{
	va_list ap;
	char *buf = NULL;
	const char *file;
	int line;
	
	struct log *l = global_log ? global_log : &default_log;

	va_start(ap, fmt);
	vstrcatf(&buf, fmt, ap);
	va_end(ap);
	
	line = config_setting_source_line(cfg);
	file = config_setting_source_file(cfg);
	if (!file)
		file = config_setting_get_hook(config_root_setting(cfg->config));

	log_print(l, LOG_LVL_ERROR, "%s in %s:%u", buf, file, line);

	free(buf);
	die();
}
