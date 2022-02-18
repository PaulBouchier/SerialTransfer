#include "SerialTransfer.h"


/*! \brief Advanced initializer for SerialTransfer objects

  \param [in] _port Serial port to communicate over
  \param [in] configs Struct that holds config values for all possible initialization parameters
*/
void SerialTransfer::begin(Stream& _port, const configST configs)
{
	port = &_port;
	packet.begin(configs);
}

/*! \brief Initialize SerialTransport object

  Simple initializer for the SerialTransfer Class. Additional configuration options are available
  from the advanced initialization begin() method.
  \param [in] _port Serial port to communicate over
  \param [in] _debug Whether or not to print error messages. Default: true
  \param [in] _debugPort Serial port to print error messages. Default: Serial
  \param [in] timeout Number of ms to wait before declaring packet parsing timeout. Default: 50ms
*/
void SerialTransfer::begin(Stream& _port, const bool _debug, Stream& _debugPort, uint32_t _timeout)
{
	port    = &_port;
	timeout = _timeout;
	packet.begin(_debug, _debugPort, _timeout);
}

/*! \brief Send the packet which was previously populated by txObj() calls to the serial port

  COBS-Encode and write a specified number of bytes in packetized form to the serial link
  after populating the packetID, CRC, and other header fields.

  \param messageLen Number of values in txBuff to send as the payload in the next packet
  \param packetID The packet 8-bit identifier
  \return numBytesIncl Number of payload bytes included in packet
*/
uint8_t SerialTransfer::sendData(const uint16_t& messageLen, const uint8_t packetID)
{
	uint8_t numBytesIncl;

	numBytesIncl = packet.constructPacket(messageLen, packetID);
	port->write(packet.preamble, sizeof(packet.preamble));
	port->write(packet.txBuff, numBytesIncl);
	port->write(packet.postamble, sizeof(packet.postamble));

	return numBytesIncl;
}


/*! \brief Handle incoming data and report if a complete packet is available

  Parses and decodes incoming serial data, analyzes packet contents,
  and reports errors/successful packet reception. If callbacks have been
  configured they are called from within this function.
  \return bytesRead - Number of bytes in RX buffer
*/
uint8_t SerialTransfer::available()
{
	bool    valid   = false;
	uint8_t recChar = 0xFF;

	if (port->available())
	{
		valid = true;

		while (port->available())
		{
			recChar = port->read();

			bytesRead = packet.parse(recChar, valid);
			status    = packet.status;

			if (status != CONTINUE)
			{
				if (status < 0)
					reset();

				break;
			}
		}
	}
	else
	{
		bytesRead = packet.parse(recChar, valid);
		status    = packet.status;

		if (status < 0)
			reset();
	}

	return bytesRead;
}


/*! \brief Report whether a complete packet is available

  Checks to see if any packets have been fully parsed. This
  is basically a wrapper around the method "available()" and
  is used primarily in conjunction with callbacks

  \return True if a full packet has been parsed and is available, false otherwise
*/
bool SerialTransfer::tick()
{
	if (available())
		return true;

	return false;
}


/*! \brief Get the packet ID of the last parsed packet

  \return ID of the last parsed packet
*/
uint8_t SerialTransfer::currentPacketID()
{
	return packet.currentPacketID();
}

/*! \brief Clear buffers and reset the object

  Clears out the tx, and rx buffers, plus resets
  the "bytes read" variable, finite state machine, etc
*/
void SerialTransfer::reset()
{
	while (port->available())
		port->read();

	packet.reset();
	status = packet.status;
}
