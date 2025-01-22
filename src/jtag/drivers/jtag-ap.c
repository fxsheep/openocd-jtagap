/***************************************************************************
 *   Copyright (C) 2019 by Peter Lawrence                                  *
 *   majbthrd@gmail.com                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/* the thunking TCL socket code at the end borrows heavily from remote_bitbang.c
 * which is Copyright (C) 2011 by Richard Uhler
 */

/* this code has three sections in this order:
 * 1) the JTAG-AP interface implementation
 * 2) struct jtag_interface and command handler definitions
 * 3) thunking code that maps DAP operations to TCL "apreg" operations
 */

/*
 * JTAG-AP is part of ARM's ADIv5 specification (IHI 0031).
 *
 * JTAG-AP allows up to eight distinct legacy JTAG chains to be individually
 * accessed over a CoreSight bus.  Nominally, this would be one ARM TAP
 * on each port, but this is not a hard requirement.
 *
 * To map this into OpenOCD's architecture, I resorted to running two
 * instances of OpenOCD.  The first connects to the target hardware (and uses
 * a mem_ap target, if needed, to pacify OpenOCD's need for a target); the
 * second uses this source code's interface implementation and connects to
 * the first via its TCL port and issues many "apreg" commands.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include "../../target/arm_adi_v5.h"

/* prototypes for code in Section 3 */
static int jtag_ap_thunk_init(void);
static int jtag_ap_thunk_quit(void);
static int jtag_ap_thunk_dap_read(uint32_t reg, uint32_t *value);
static int jtag_ap_thunk_dap_write(uint32_t reg, uint32_t value);

static uint32_t jtag_ap_thunk_jtagport;

/*
 * Section 1: the JTAG-AP interface implementation
 */

static int jtag_ap_init(void)
{
	LOG_INFO("Initializing jtag_ap driver");

	if (ERROR_OK != jtag_ap_thunk_init())
		return ERROR_FAIL;

	/* there must be one and only one JTAG-AP port selected */
	uint32_t psel = 1UL << jtag_ap_thunk_jtagport;
	jtag_ap_thunk_dap_write(JTAG_AP_REG_PSEL, psel);

	LOG_INFO("jtag_ap driver initialized");

	return ERROR_OK;
}

static int jtag_ap_quit(void)
{
	if (ERROR_OK != jtag_ap_thunk_quit())
		return ERROR_FAIL;

	LOG_INFO("jtag_ap interface quit");
	return ERROR_OK;
}

static void jtag_ap_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void jtag_ap_wait_until_ready(int clear_data)
{
	uint32_t status = 0;
	uint32_t data;

	for (;;) {
		/* read current CSW */
		if (ERROR_OK != jtag_ap_thunk_dap_read(JTAG_AP_REG_CSW, &status))
			break;

		/* loop until SERACTV bit of CSR is no longer set */
		if (status & CSW_SERACTV_MASK)
			continue;

		/* if clear_data is set, flush any existing data */
		if (clear_data && (status & CSW_RFIFOCNT_MASK)) {
			/* read an outstanding byte from Response FIFO */
			jtag_ap_thunk_dap_read(JTAG_AP_REG_BRFIFO1, &data);
			continue;
		}

		break;
	}
}

static void jtag_ap_state_execute(uint32_t tms_scan, uint32_t tms_scan_bits)
{
	while (tms_scan_bits) {
		/* each JTAG-AP TMS command byte can encode up to 5 TMS bits */
		/* see ARM IHI 0031 Section 9.2.1 for further details */
		uint32_t chunk_size = (tms_scan_bits > 5) ? 5 : tms_scan_bits;
		uint32_t encode_len = 1UL << chunk_size;
		uint32_t command = (tms_scan & (encode_len - 1)) | encode_len;

		jtag_ap_wait_until_ready(1);
		jtag_ap_thunk_dap_write(JTAG_AP_REG_BWFIFO1, command);

		tms_scan >>= chunk_size;
		tms_scan_bits -= chunk_size;
	}
}

static void jtag_ap_state_move(int skip)
{
	uint32_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint32_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	/* chop off the prescribed number of bits at the beginning */
	tms_scan >>= skip;
	tms_scan_bits -= skip;

	jtag_ap_state_execute(tms_scan, tms_scan_bits);

	tap_set_state(tap_get_end_state());
}

static void jtag_ap_stableclocks(int num_cycles)
{
	uint32_t tms_scan = (tap_get_state() == TAP_RESET) ? 0xFFFFFFFF : 0;

	while (num_cycles) {
		/* 30 rather than 32 bits to be an even multiple of five */
		uint32_t chunk_size = (num_cycles > 30) ? 30 : num_cycles;
		jtag_ap_state_execute(tms_scan, chunk_size);
		num_cycles -= chunk_size;
	}
}

static void jtag_ap_cmd_stableclocks(struct jtag_command *cmd)
{
	jtag_ap_stableclocks(cmd->cmd.runtest->num_cycles);
}

static void jtag_ap_cmd_runtest(struct jtag_command *cmd)
{
	tap_state_t saved_end_state = tap_get_end_state();

	/* Only do a state_move when we're not already in IDLE. */
	if (TAP_IDLE != tap_get_state()) {
		jtag_ap_end_state(TAP_IDLE);
		jtag_ap_state_move(0);
	}

	jtag_ap_stableclocks(cmd->cmd.runtest->num_cycles);

	/* Finish in end_state. */
	jtag_ap_end_state(saved_end_state);

	if (tap_get_state() != tap_get_end_state())
		jtag_ap_state_move(0);
}

static void jtag_ap_cmd_statemove(struct jtag_command *cmd)
{
	jtag_ap_end_state(cmd->cmd.statemove->end_state);
	jtag_ap_state_move(0);
}

static void jtag_ap_cmd_pathmove(struct jtag_command *cmd)
{
	int num_states = cmd->cmd.pathmove->num_states;
	tap_state_t *path = cmd->cmd.pathmove->path;
	uint32_t tms_scan_bits = 0;
	uint32_t tms_scan = 0;

	while (num_states--) {
		if (tap_state_transition(tap_get_state(), true) == *path)
			tms_scan |= 1UL << tms_scan_bits;
		else if (tap_state_transition(tap_get_state(), false) != *path) {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				tap_state_name(tap_get_state()), tap_state_name(*path));
			exit(-1);
		}

		tms_scan_bits++;
		tap_set_state(*path);
		path++;
	}

	jtag_ap_state_execute(tms_scan, tms_scan_bits);

	tap_set_end_state(tap_get_state());
}

static int jtag_ap_cmd_scan(struct jtag_command *cmd)
{
	int scan_size, chunk_size;
	uint8_t *buffer, *buffer_ptr;
	int retval = ERROR_OK;
	enum scan_type scan_type = jtag_scan_type(cmd->cmd.scan);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT) {
			jtag_ap_end_state(TAP_IRSHIFT);
			jtag_ap_state_move(0);
		}
	} else {
		if (tap_get_state() != TAP_DRSHIFT) {
			jtag_ap_end_state(TAP_DRSHIFT);
			jtag_ap_state_move(0);
		}
	}

	for (buffer_ptr = buffer; scan_size; scan_size -= chunk_size, buffer_ptr++) {

		/* JTAG-AP TDI_TDO packets are 2 or more bytes in length */
		/* see ARM IHI 0031 Section 9.2.2 for further details */

		/* the following is a 3-byte packet consisting of:
		 * opcode byte
		 * 'normal format' length byte
		 * data byte with up to 8 bits of TDI
		 */

		uint32_t command = 0x84; /* TDI_TDO opcode - RTDO set */

		chunk_size = scan_size;
		if (chunk_size > 8)
			chunk_size = 8;
		else
			command |= 1UL << 3; /* TMS HIGH on last cycle */

		/* TDI_TDO 'normal format' length byte */
		command |= (chunk_size - 1) << 8;

		/* data byte */
		if (SCAN_IN != scan_type)
			command |= (uint32_t)*buffer_ptr << 16;

		jtag_ap_wait_until_ready(1);
		jtag_ap_thunk_dap_write(JTAG_AP_REG_BWFIFO3, command);
		jtag_ap_wait_until_ready(0);
		jtag_ap_thunk_dap_read(JTAG_AP_REG_BRFIFO1, &command);

		/* chunk_size bits of TDO have been returned */
		*buffer_ptr = (uint8_t)command;
	}

	if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK) {
		retval = ERROR_JTAG_QUEUE_FAILED;
		goto bail;
	}

bail:
	if (buffer)
		free(buffer);

	if (tap_get_state() != cmd->cmd.scan->end_state) {
		jtag_ap_end_state(cmd->cmd.scan->end_state);
		/* in this special case, we've already done the first TMS=1 */
		jtag_ap_state_move(1);
	}

	return retval;
}

static void jtag_ap_cmd_reset(struct jtag_command *cmd)
{
	/* this reset operation procedure is documented under the heading:
	 * 'Resetting JTAG devices' in ARM IHI 0031 Section 12.2.1
	 */

	/* assert TRST_OUT */
	jtag_ap_thunk_dap_write(JTAG_AP_REG_CSW, CSW_TRST_OUT_MASK);
	/* drive sequence of at least five TMS=1 clocks */
	jtag_ap_thunk_dap_write(JTAG_AP_REG_BWFIFO1, 0x3F);
	/* de-assert TRST_OUT */
	jtag_ap_thunk_dap_write(JTAG_AP_REG_CSW, 0);
}

static void jtag_ap_cmd_sleep(struct jtag_command *cmd)
{
	jtag_sleep(cmd->cmd.sleep->us);
}

static int jtag_ap_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_STABLECLOCKS:
			jtag_ap_cmd_stableclocks(cmd);
			break;
		case JTAG_RUNTEST:
			jtag_ap_cmd_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			jtag_ap_cmd_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			jtag_ap_cmd_pathmove(cmd);
			break;
		case JTAG_SCAN:
			jtag_ap_cmd_scan(cmd);
			break;
		case JTAG_RESET:
			jtag_ap_cmd_reset(cmd);
			break;
		case JTAG_SLEEP:
			jtag_ap_cmd_sleep(cmd);
			break;
		default:
			LOG_ERROR("BUG: Unknown JTAG command type encountered.");
			return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

static int jtag_ap_queue(struct jtag_command *cmd_queue)
{
	int ret;
	struct jtag_command *cmd = cmd_queue;

	while (NULL != cmd) {
		ret = jtag_ap_command(cmd);

		if (ERROR_OK != ret)
			return ret;

		cmd = cmd->next;
	}

	return ERROR_OK;
}

/*
 * Section 2: struct jtag_interface and command handler definitions
 */

/* prototype functions in thunking code further below */
COMMAND_HANDLER(jtag_ap_handle_thunk_port_command);
COMMAND_HANDLER(jtag_ap_handle_thunk_dap_instance);
COMMAND_HANDLER(jtag_ap_handle_thunk_apsel);
COMMAND_HANDLER(jtag_ap_handle_thunk_jtagport);

static const struct command_registration jtag_ap_subcommand_handlers[] = {
	{
		.name = "port",
		.handler = jtag_ap_handle_thunk_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the port to use to connect to the remote TCL server.",
		.usage = "port_number",
	},
	{
		.name = "dap_instance",
		.handler = jtag_ap_handle_thunk_dap_instance,
		.mode = COMMAND_CONFIG,
		.help = "name of dap instance (i.e. \"foo.dap\") on the remote server",
		.usage = "dap_instance",
	},
	{
		.name = "apsel",
		.handler = jtag_ap_handle_thunk_apsel,
		.mode = COMMAND_CONFIG,
		.help = "number of dap AP on the remote server providing JTAG-AP",
		.usage = "ap_num",
	},
	{
		.name = "jtagport",
		.handler = jtag_ap_handle_thunk_jtagport,
		.mode = COMMAND_CONFIG,
		.help = "number(0-7) of JTAG port to use on JTAG-AP",
		.usage = "jtag_port",
	},
	COMMAND_REGISTRATION_DONE,
};

static const struct command_registration jtag_ap_command_handlers[] = {
	{
		.name = "jtag_ap",
		.mode = COMMAND_ANY,
		.help = "JTAG-AP commands",
		.chain = jtag_ap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface jtag_ap_interface = {
//	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = jtag_ap_queue,
};

struct adapter_driver jtag_ap_adapter_driver = {
	.name = "jtag-ap",
	.jtag_ops = &jtag_ap_interface,
	.transports = jtag_only,
	.commands = jtag_ap_command_handlers,
	.init = jtag_ap_init,
	.quit = jtag_ap_quit,
};

/*
 * Section 3: thunking code that maps DAP operations to TCL "apreg" operations
 */

#ifndef _WIN32
#include <sys/un.h>
#include <netdb.h>
#endif

static char *jtag_thunk_ap_port;
static char *jtag_ap_thunk_dap_instance;
static const char *jtag_ap_default_dap_instance = "portal.dap";
static uint32_t jtag_ap_thunk_apsel;

static FILE *jtag_ap_thunk_file;
static int jtag_ap_thunk_fd;

COMMAND_HANDLER(jtag_ap_handle_thunk_port_command)
{
	if (CMD_ARGC == 1) {
		uint16_t port;
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], port);
		free(jtag_thunk_ap_port);
		jtag_thunk_ap_port = port == 0 ? NULL : strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(jtag_ap_handle_thunk_dap_instance)
{
	if (CMD_ARGC == 1) {
		free(jtag_ap_thunk_dap_instance);
		jtag_ap_thunk_dap_instance = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(jtag_ap_handle_thunk_apsel)
{
	if (CMD_ARGC == 1) {
		uint32_t apsel;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		if (apsel > DP_APSEL_MAX)
			return ERROR_COMMAND_SYNTAX_ERROR;
		jtag_ap_thunk_apsel = apsel;
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(jtag_ap_handle_thunk_jtagport)
{
	if (CMD_ARGC == 1) {
		uint32_t jtagport;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], jtagport);
		if (jtagport > 7)
			return ERROR_COMMAND_SYNTAX_ERROR;
		jtag_ap_thunk_jtagport = jtagport;
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static int jtag_ap_thunk_init(void)
{
	struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
	struct addrinfo *result, *rp;

	LOG_INFO("Connecting to localhost:%s", jtag_thunk_ap_port);

	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(NULL, jtag_thunk_ap_port, &hints, &result);
	if (s != 0) {
		LOG_ERROR("getaddrinfo: %s\n", gai_strerror(s));
		return ERROR_FAIL;
	}

	/* getaddrinfo() returns a list of address structures.
	 Try each address until we successfully connect(2).
	 If socket(2) (or connect(2)) fails, we (close the socket
	 and) try the next address. */

	for (rp = result; rp != NULL ; rp = rp->ai_next) {
		jtag_ap_thunk_fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (jtag_ap_thunk_fd == -1)
			continue;

		if (connect(jtag_ap_thunk_fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break; /* Success */

		close(jtag_ap_thunk_fd);
	}

	freeaddrinfo(result); /* No longer needed */

	if (rp == NULL) { /* No address succeeded */
		LOG_ERROR("Failed to connect: %s", strerror(errno));
		return ERROR_FAIL;
	}

	if (jtag_ap_thunk_fd < 0)
		return ERROR_FAIL;

	jtag_ap_thunk_file = fdopen(jtag_ap_thunk_fd, "w+");
	if (jtag_ap_thunk_file == NULL) {
		LOG_ERROR("fdopen: failed to open write stream");
		close(jtag_ap_thunk_fd);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int jtag_ap_thunk_quit(void)
{
	if (EOF == fflush(jtag_ap_thunk_file)) {
		LOG_ERROR("fflush: %s", strerror(errno));
		return ERROR_FAIL;
	}

	/* We only need to close one of the FILE*s, because they both use the same */
	/* underlying file descriptor. */
	if (EOF == fclose(jtag_ap_thunk_file)) {
		LOG_ERROR("fclose: %s", strerror(errno));
		return ERROR_FAIL;
	}

	free(jtag_thunk_ap_port);

	return ERROR_OK;
}

static int jtag_ap_thunk_dap_write(uint32_t reg, uint32_t value)
{
	char byte;
	ssize_t count;
	const char *dap_instance = (jtag_ap_thunk_dap_instance) ?
		jtag_ap_thunk_dap_instance : jtag_ap_default_dap_instance;

	/* generate Ctrl-Z terminated apreg write operation */
	if (EOF == fprintf(jtag_ap_thunk_file, "%s apreg %u %d 0x%x\x1A",
		dap_instance, jtag_ap_thunk_apsel, reg, value)) {
		LOG_ERROR("jtag_ap_thunk_dap_write fprintf: %s", strerror(errno));
		return ERROR_FAIL;
	}

	fflush(jtag_ap_thunk_file);

	/* look for a Ctrl-Z, ignoring any intermediate data */
	for (;;) {
		count = read(jtag_ap_thunk_fd, &byte, 1);

		if (count <= 0) {
			LOG_ERROR("jtag_ap_thunk_dap_write read: %s (%d)",
					strerror(errno), errno);
			return ERROR_FAIL;
		}

		if ('\x1A' == byte)
			break;
	}

	return ERROR_OK;
}

static int jtag_ap_thunk_dap_read(uint32_t reg, uint32_t *value)
{
	char buffer[64];
	ssize_t count, index;
	const char *dap_instance = (jtag_ap_thunk_dap_instance) ?
		jtag_ap_thunk_dap_instance : jtag_ap_default_dap_instance;

	/* generate Ctrl-Z terminated apreg read operation */
	if (EOF == fprintf(jtag_ap_thunk_file, "%s apreg %u %d\x1A",
		dap_instance, jtag_ap_thunk_apsel, reg)) {
		LOG_ERROR("jtag_ap_thunk_dap_read fprintf: %s", strerror(errno));
		return ERROR_FAIL;
	}

	fflush(jtag_ap_thunk_file);

	/* look for a Ctrl-Z, collecting intermediate data in 'buffer' */
	for (index = 0; index < (ssize_t)sizeof(buffer); index++) {
		count = read(jtag_ap_thunk_fd, buffer + index, 1);

		if (count <= 0) {
			LOG_ERROR("jtag_ap_thunk_dap_read read: %s (%d)",
					strerror(errno), errno);
			return ERROR_FAIL;
		}

		if ('\x1A' == buffer[index]) {
			buffer[index] = '\0';
			break;
		}
	}

	if (value)
		*value = strtoul(buffer, NULL, 0);

	return ERROR_OK;
}

