# Custom UART Communication Protocol

This project uses a custom UART packet format to send data between the HV MCU and the LV display MCU.

The protocol is designed to be:

* simple to parse
* compact
* easy to extend with new message IDs
* robust against partial UART reads by using a start byte and checksum

---

## Packet Structure

Each UART message has the following byte layout:

| Field |    Size | Description                                     |
| ----- | ------: | ----------------------------------------------- |
| SOF   |  1 byte | Start of frame marker, always `0xA5`            |
| ID    | 2 bytes | Message ID, little-endian                       |
| DATA  | 2 bytes | Fixed 2-byte payload                            |
| CRC   |  1 byte | XOR checksum of all previous bytes in the frame |

---

## Byte Layout

```
+------+------+------+------+------+------+
| SOF  | ID_L | ID_H | DATA0| DATA1| CRC  |
+------+------+------+------+------+------+
   1B     1B     1B     1B     1B    1B
```

---

## Field Descriptions

### SOF (Start of Frame)

* Fixed value: `0xA5`
* Marks the beginning of a new packet
* Allows the parser to resynchronize if bytes are lost or corrupted

---

### ID (Message Identifier)

* 16-bit value (little-endian)
* Defines how the payload should be interpreted

Example:

```
#define UART_ID_VOLTAGE 0x0001
#define UART_ID_CURRENT 0x0002
```

---

### DATA (Payload)

* Fixed 2-byte payload
* Interpretation depends on the message ID
* Values are encoded using packed decimal digits (BCD-style)

Each byte contains two decimal digits:

```
DATA0 = [d0 | d1]
DATA1 = [d2 | d3]
```

Example:

```
DATA0 = (d0 << 4) | d1
DATA1 = (d2 << 4) | d3
```

Typical formats:

* Voltage: `xxx.x` (e.g. 250.3 → digits 2,5,0,3)
* Current: `xx.xx` (e.g. 7.52 → digits 0,7,5,2)

---

### CRC (Checksum)

* 1-byte XOR checksum of all previous bytes:

  * SOF
  * ID_L
  * ID_H
  * DATA0
  * DATA1

Example:

```
uint8_t crc = xor_crc(buf, 5);
```

* Used for basic error detection
* Frames with invalid CRC are discarded

---

## Example Frames

### Voltage = 250.3 V

* ID = `0x0001`
* Digits: `2 5 0 3`
* DATA0 = `0x25`
* DATA1 = `0x03`

Frame:

```
A5 01 00 25 03 CRC
```

---

### Current = 7.52 A

* ID = `0x0002`
* Digits: `0 7 5 2`
* DATA0 = `0x07`
* DATA1 = `0x52`

Frame:

```
A5 02 00 07 52 CRC
```

---

## Parsing Strategy

The receiver processes incoming bytes using a state machine:

1. Wait for `SOF (0xA5)`
2. Read `ID_L`
3. Read `ID_H`
4. Read `DATA0`
5. Read `DATA1`
6. Read `CRC`
7. Validate checksum
8. If valid → output message
9. If invalid → discard and wait for next SOF

---

## Notes

* This protocol currently uses a fixed 2-byte payload
* Designed for fast, low-overhead MCU-to-MCU communication
* Can be extended in future (e.g. adding a length field) if needed
