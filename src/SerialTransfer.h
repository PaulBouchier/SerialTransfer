#pragma once
#include "Arduino.h"
#include "Packet.h"

/*! \file SerialTransfer.hD
    \brief File declaring the SerialTransfer class and implementing some methods
*/

/*! \class SerialTransfer
 * \brief Transport protocol for passing packetized data over a serial link.
 *
 * The SerialTransfer class implements a protocol that transfers packetized data
 * fast and reliably via Serial, I2C, and SPI interfaces. It runs on Arduino,
 * where it works with SerialTransfer on other Arduinos, or with pySerialTransfer
 * on a computer running Python.
 */
class SerialTransfer
{
  public: // <<---------------------------------------//public
	//! \brief Class instance for low-level packet parsing and handling
	Packet  packet;

	/*!
	\bug Class members bytesRead and status are not valid during callbacks, and are duplicates of the
	corresponding variables in class packet. They should be removed, and packet variables
	should be recommended for use by clients of this class instead.
	*/

	//! \brief Number of bytes read during call to available().
	//!
	//! Not valid during a callback - use 
	//! packet.bytesRead instead.
	uint8_t bytesRead = 0;

	//! \brief Packet fsm state. See packet class documentation for values.
	//!
	//! Not valid during a callback - use packet.status instead
	int8_t  status    = 0;


	void    begin(Stream& _port, const configST configs);


	/*! \brief Initialize SerialTransport object
	 *
	 * \param [in] _port Reference to the port object (e.g. Serial1) that must have been initialized.
	 * \param [in] _debug Passed to Packet.begin. See Packet class documentation
	 * \param [in] debugPort Passed to Packet.begin. See Packet class documentation
	 * \param [in] _timeout Passed to Packet.begin. See Packet class documentation
	 */
	void    begin(Stream& _port, const bool _debug = true, Stream& _debugPort = Serial, uint32_t _timeout = DEFAULT_TIMEOUT);
	uint8_t sendData(const uint16_t& messageLen, const uint8_t packetID = 0);
	uint8_t available();
	bool    tick();
	uint8_t currentPacketID();
	void    reset();


	/*! \brief Encode the object "val", passed by reference, into the transmit buffer

	  Stuffs "len" number of bytes of an arbitrary object (byte, int,
	  float, double, struct, etc...) into the transmit buffer (txBuff)
	  starting at the index as specified by the argument "index". Performs
	  COBS byte-stuffing as it puts data in the transmit buffer.

	  \param [in] val Reference to the object to be copied to the transmit buffer (txBuff)
	  \param [in] index Starting index of the object within the transmit buffer (txBuff)
	  \param [in] len - Number of bytes of the object "val" to transmit
	  \return Index of the transmit buffer (txBuff) byte that directly follows the bytes processed
	  by the calling of this member function. The return is intended to be used as index by
	  the next call to txObj (if one exists).
	*/
	template <typename T>
	uint16_t txObj(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
	{
		return packet.txObj(val, index, len);
	}


	/*
	 uint16_t SerialTransfer::rxObj(const T &val, const uint16_t &index=0, const uint16_t &len=sizeof(T))
	 Description:
	 ------------
	  * Reads "len" number of bytes from the receive buffer (rxBuff)
	  starting at the index as specified by the argument "index"
	  into an arbitrary object (byte, int, float, double, struct, etc...)
	 Inputs:
	 -------
	  * const T &val - Pointer to the object to be copied into from the
	  receive buffer (rxBuff)
	  * const uint16_t &index - Starting index of the object within the
	  receive buffer (rxBuff)
	  * const uint16_t &len - Number of bytes in the object "val" received
	 Return:
	 -------
	  * uint16_t maxIndex - Index of the receive buffer (rxBuff) that directly follows the bytes processed
	  by the calling of this member function
	*/
	template <typename T>
	uint16_t rxObj(const T& val, const uint16_t& index = 0, const uint16_t& len = sizeof(T))
	{
		return packet.rxObj(val, index, len);
	}


	/*
	 uint8_t SerialTransfer::sendDatum(const T &val, const uint16_t &len=sizeof(T))
	 Description:
	 ------------
	  * Stuffs "len" number of bytes of an arbitrary object (byte, int,
	  float, double, struct, etc...) into the transmit buffer (txBuff)
	  starting at the index as specified by the argument "index" and
	  automatically transmits the bytes in an individual packet
	 Inputs:
	 -------
	  * const T &val - Pointer to the object to be copied to the
	  transmit buffer (txBuff)
	  * const uint16_t &len - Number of bytes of the object "val" to transmit
	 Return:
	 -------
	  * uint8_t - Number of payload bytes included in packet
	*/
	template <typename T>
	uint8_t sendDatum(const T& val, const uint16_t& len = sizeof(T))
	{
		return sendData(packet.txObj(val, 0, len));
	}


  private: // <<---------------------------------------//private
	Stream* port;
	uint32_t timeout;
};
