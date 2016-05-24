/*
 * Libserialport-fake
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include <mbed/serial_api.h>

#include "libserialport.h"

struct sp_port {
	serial_t uart;
	int data_bits, stop_bits;
	SerialParity parity;
	char *name, *description;
};

static struct sp_port sp_ports[5];

static void init_port(struct sp_port *port, PinName tx, PinName rx,
		char *name, char *description)
{
	serial_init(&port->uart, tx, rx);
	serial_pinout_tx(tx);

	port->data_bits = 8;
	port->stop_bits = 1;
	port->parity = ParityNone;
	port->name = name;
	port->description = description;
}

enum sp_return sp_get_port_by_name(const char *portname,
		struct sp_port **port_ptr)
{
	if (!strcmp(portname, "USB") || !strcmp(portname, "UART0")) {
		init_port(&sp_ports[0], USBTX, USBRX, "UART0", "USB UART");
		*port_ptr = &sp_ports[0];
		return SP_OK;
	}

	if (!strcmp(portname, "UART3")) {
		init_port(&sp_ports[3], PTC17, PTC16, "UART3", "Arduino UART");
		*port_ptr = &sp_ports[3];
		return SP_OK;
	}

	return SP_ERR_ARG;
}

void sp_free_port(struct sp_port *port)
{
	serial_free(&port->uart);
}

enum sp_return sp_open(struct sp_port *port, enum sp_mode flags)
{
	if (flags != SP_MODE_READ_WRITE)
		return SP_ERR_SUPP;
	else
		return SP_OK;
}

enum sp_return sp_close(struct sp_port *port)
{
	return SP_OK;
}

enum sp_return sp_set_baudrate(struct sp_port *port, int baudrate)
{
	serial_baud(&port->uart, baudrate);
	return SP_OK;
}

enum sp_return sp_set_bits(struct sp_port *port, int bits)
{
	port->data_bits = bits;
	serial_format(&port->uart, bits, port->parity, port->stop_bits);
	return SP_OK;
}

enum sp_return sp_set_stopbits(struct sp_port *port, int stopbits)
{
	port->stop_bits = stopbits;
	serial_format(&port->uart, port->data_bits, port->parity, stopbits);
	return SP_OK;
}

enum sp_return sp_set_parity(struct sp_port *port, enum sp_parity parity)
{
	switch (parity) {
	case SP_PARITY_NONE:
		port->parity = ParityNone;
		break;
	case SP_PARITY_ODD:
		port->parity = ParityOdd;
		break;
	case SP_PARITY_EVEN:
		port->parity = ParityEven;
		break;
	default:
		return SP_ERR_SUPP;
	}

	serial_format(&port->uart, port->data_bits,
			port->parity, port->stop_bits);
	return SP_OK;
}

enum sp_return sp_set_flowcontrol(
		struct sp_port *port, enum sp_flowcontrol flow)
{
	if (flow != SP_FLOWCONTROL_NONE)
		return SP_ERR_SUPP;

#if 0
	/* This function is in serial_api.h, but it's not implemented anywhere */
	serial_set_flow_control(&port->uart, FlowControlNone, NC, NC);
#endif
	return SP_OK;
}

enum sp_return sp_flush(struct sp_port *port, enum sp_buffer buffers)
{
	if (buffers == SP_BUF_BOTH)
		serial_clear(&port->uart);
	return SP_OK;
}

char *sp_get_port_name(const struct sp_port *port)
{
	return port->name;
}

char *sp_get_port_description(const struct sp_port *port)
{
	return port->description;
}

enum sp_return sp_blocking_write(struct sp_port *port, const void *buf,
		size_t count, unsigned int timeout_ms)
{
	size_t i;
	const unsigned char *ptr = buf;

	for (i = 0; i < count; i++)
		serial_putc(&port->uart, (int) ptr[i]);

	return count;
}

enum sp_return sp_blocking_read_next(struct sp_port *port, void *buf,
		size_t count, unsigned int timeout_ms)
{
	if (count == 0)
		return SP_ERR_ARG;

	*(char *) buf = (char) serial_getc(&port->uart);
	return 1;
}

int sp_last_error_code(void)
{
	return 42;
}
