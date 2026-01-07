#ifndef MICROSTIM_H_
#define MICROSTIM_H_
/**
 * File: uart_comms.h
 * 
 * This header file centralises uart communication protocol 
 * configurations for the microstimulation system, ensuring consistency 
 * and simplifying updates across the project.
 * 
 *
 * ## Revision History
 * - v0.1: Initial file.
 *
 * @author George Brayshaw
 * @date 10/12/2025
 * @version 0.1
 */
//--------------------------------------------------------------------------------------------------
// Includes
//--------------------------------------------------------------------------------------------------
#include "Arduino.h"      // Arduino library


//--------------------------------------------------------------------------------------------------
// Communication Settings
//--------------------------------------------------------------------------------------------------
#define TEENSY_UART_BAUD_RATE 9600
#define CONTROL_BOARD_UART_BAUD_RATE 9600
#define UART_BUFFER_SIZE 64 // Length of the UART string used to communicate stimulation param changes 

// Heartbeat timeout used by both control and stim boards (milliseconds)
#ifndef HEARTBEAT_TIMEOUT_MS
#define HEARTBEAT_TIMEOUT_MS 300
#endif

//--------------------------------------------------------------------------------------------------
// Enumerations and Typedefs
//--------------------------------------------------------------------------------------------------
typedef enum MODE{
    GND,        // System grounded
    STIMULATE,  // Stimulation mode
    READ,       // Read mode
} Modes;

// TODO: Implement error codes
typedef enum ERROR_CODES{
    ERROR_NONE,   // No error
} ErrorCodes;


/*--------------------------------------------------------------------------------------------------
// UART Protocol Definition
//--------------------------------------------------------------------------------------------------
    The UART communication protocol for Pi <-> Control <-> Stimulator Board is defined as follows:
   ------------------------------------------------------------
   Packet structure (total length = variable):

   [0]  START_BYTE        (0xAA)
   [2]  MSG_TYPE          (enum)
   [3]  PAYLOAD_LENGTH    (N bytes)
   [4..N+3] PAYLOAD       (0–255 bytes)
   [N+4] CHECKSUM         (XOR of MSG_TYPE, PAYLOAD_LENGTH, PAYLOAD[])
   [N+5] END_BYTE         (0x55)

   This protocol:
     • is byte-aligned (simple for Arduino Serial reads)
     • uses framing markers
     • supports arbitrary payloads
     • includes error detection via XOR checksum
     • supports all command categories needed
   ============================================================
*/
// UART framing bytes and protocol version (compile-time constants)
#define UART_START_BYTE   0xAA
#define UART_END_BYTE     0x55
#define UART_PROTOCOL_VERSION 0x01
#define UART_MAX_PAYLOAD 64 // Maximum safe payload for Arduino Serial (compile-time constant)

// UART message types
enum UART_MSG_TYPE : uint8_t {
    UART_MSG_NONE          = 0x00,    // No/invalid message (parser failure)
    UART_MSG_HEARTBEAT      = 0x01,   // Heartbeat packet
    UART_MSG_SHUTDOWN       = 0x02,   // Emergency shutdown request
    UART_MSG_STIM           = 0x03,   // Update internal values/configurations
    UART_MSG_RECORD         = 0x04,   // Update stimulation amplitude
    UART_MSG_UPDATE_WIDTH   = 0x05,   // Update internal values/configurations
    UART_MSG_UPDATE_AMP     = 0x06,   // Update stimulation amplitude
    UART_MSG_ACK            = 0x07,   // Handshake acknowledgement packet
    UART_MSG_ERROR          = 0x08,   // Protocol error report
    UART_MSG_GET_PARAMS     = 0x09,   // Request stim parameters (no payload)
    UART_MSG_SEND_WIDTH     = 0x0B,   // Send width only (payload: int32 BE)
    UART_MSG_SEND_AMP       = 0x0C,   // Send amplitude only (payload: float32 BE)
    UART_MSG_START          = 0x0D    // Exit safe state (no payload)
};

// Payload types indicate how payload bytes should be interpreted
enum PAYLOAD_TYPE : uint8_t {
    PAYLOAD_NONE   = 0x00,
    PAYLOAD_INT32  = 0x01, // 4 bytes, big-endian int32
    PAYLOAD_FLOAT32= 0x02, // 4 bytes, IEEE-754 big-endian
    PAYLOAD_BYTES  = 0x03  // Raw bytes
};

// Standardized shutdown source codes carried in UART_MSG_SHUTDOWN payload (PAYLOAD_INT32)
// These allow each component (Pi, Control, Stim) to see where the shutdown originated
// and for intermediaries to relay that information downstream.
enum SHUTDOWN_SRC : int32_t {
    SHUTDOWN_SRC_UNKNOWN = 0,
    SHUTDOWN_SRC_CONTROL = 1,
    SHUTDOWN_SRC_STIM    = 2,
    SHUTDOWN_SRC_PI      = 3
};

//--------------------------------------------------------------------------------------------------
// Function Declarations
//--------------------------------------------------------------------------------------------------
uint8_t computeCRC(UART_MSG_TYPE messageType, PAYLOAD_TYPE payloadType, const uint8_t *payload, uint8_t len);

// Build packet. 'payloadType' defaults to PAYLOAD_NONE for backwards compatibility.
// Packet layout: [START][MSG_TYPE][PAYLOAD_TYPE][LENGTH][PAYLOAD...][CRC][END]
String buildPacket(UART_MSG_TYPE type, const uint8_t *payload, size_t length, PAYLOAD_TYPE payloadType = PAYLOAD_NONE);

// Convenience overloads: build a packet from a typed scalar value.
// These serialize values to big-endian representation according to payload type.
String buildPacket(UART_MSG_TYPE messageType, int32_t value);
String buildPacket(UART_MSG_TYPE messageType, float value);

// Parse packet. Returns the parsed `UART_MSG_TYPE` on success, or 0 on error.
// The function fills `*pTypeOut`, copies payload bytes into `payloadOut`
// and sets `*lenOut` to the payload length. Caller provides pointers to
// their own globals.
// Signature: parsePacket(buffer, bufferLen, &pType, payloadBuf, &payloadLen)
UART_MSG_TYPE parsePacket(const uint8_t *buffer, size_t bufferLen,
                          PAYLOAD_TYPE *pTypeOut,
                          uint8_t *payloadOut);

// -----------------------------------------------------------------------------
// Payload Parsing Helpers
// -----------------------------------------------------------------------------
// Parse `payload` according to `pType`. Supports both ASCII numeric strings
// (e.g., "123", "3.14") and 4-byte big-endian binary encodings.
// - PAYLOAD_FLOAT32: parses float32 or ASCII float
// - PAYLOAD_INT32: parses int32 or ASCII int
// - PAYLOAD_NONE / PAYLOAD_BYTES: attempts ASCII numeric parse
// On success writes parsed values to `outFloat` and/or `outInt` and returns true.
bool parsePayloadValue(const uint8_t *payload,
                       size_t payloadLen,
                       PAYLOAD_TYPE pType,
                       float *outFloat,
                       int32_t *outInt);

// -----------------------------------------------------------------------------
// Streaming parser API
// A small stateful parser that accumulates incoming bytes and extracts the
// next valid packet (if any). This allows sketches to feed serial data as it
// arrives and receive complete, CRC-validated packets without having to manage
// framing and resynchronisation themselves.

// Opaque parser state. Users should allocate this and pass it to the parser
// functions. Size is kept small to avoid large stack usage on MCUs.
typedef struct {
    uint8_t buf[256];
    size_t len;
} UartStreamParser;

// Initialise parser state
void streamParserInit(UartStreamParser *p);

// Feed bytes into the parser. If a valid packet is found it is returned via
// out parameters and the function returns true. `consumed` will contain the
// number of bytes removed from the internal buffer (useful for callers that
// want to track stream progress). The raw packet (including START/CRC/END if
// present) is copied into `rawOut` and its length into `rawLenOut` when
// non-NULL. `payloadOut` will be nul-terminated when possible.
bool streamParserFeed(UartStreamParser *p,
                      const uint8_t *data, size_t dataLen,
                      size_t *consumed,
                      UART_MSG_TYPE *msgTypeOut,
                      PAYLOAD_TYPE *pTypeOut,
                      uint8_t *payloadOut,
                      size_t *payloadLenOut,
                      uint8_t *rawOut,
                      size_t *rawLenOut);

// -----------------------------------------------------------------------------
// Convenience helper: read packet directly from a Stream (e.g., Serial1)
// This function reads available bytes from `stream`, feeds them into `parser`
// and returns the parsed `UART_MSG_TYPE` when a valid packet is found.
// On success it fills the out parameters (pTypeOut, payloadOut, payloadLenOut,
// rawOut/rawLenOut) and returns the parsed msg type. On no packet available
// it returns `UART_MSG_NONE`.
UART_MSG_TYPE readPacketFromStream(Stream &stream,
                                   UartStreamParser *parser,
                                   PAYLOAD_TYPE *pTypeOut,
                                   uint8_t *payloadOut,
                                   size_t *payloadLenOut,
                                   uint8_t *rawOut,
                                   size_t *rawLenOut);

// -----------------------------------------------------------------------------
// Link maintenance helpers
// -----------------------------------------------------------------------------
// Drain any bytes from a Stream RX buffer (non-blocking)
void uartDrainInput(Stream &stream);
// Reset a stream parser to an empty state
void uartResetParser(UartStreamParser *parser);
// Convenience: drain input and reset parser in one call
void uartFlushLink(Stream &stream, UartStreamParser *parser);

//--------------------------------------------------------------------------------------------------
// EOF
//--------------------------------------------------------------------------------------------------
#endif