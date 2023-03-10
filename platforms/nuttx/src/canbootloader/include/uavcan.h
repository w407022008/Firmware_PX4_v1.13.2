/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once
/*
 * Uavcan protocol CAN ID formats and Tail byte component definitions
 *
 * Most all the constants below are auto generated by the compiler in the
 * section above.

 * For all UAVCAN_BIT_DEFINE({item name}, lsb_pos, length) entries
 * it defines the following enumeration constants:
 *
 * Mask{item name}      - The mask that is positioned at the lsb and
 *                        length long
 * BitPos{item name}    - The bit position of the lsb
 * Length{item name}    - The number of bits in the field
 *
 *
 * For the UAVCAN_DSDL_TYPE_DEF(name, dtid, service, signature, packsize,
 *                              mailbox, fifo, inbound, outbound)
 * it defines:
 *
 *  uavcan dtid definitions are in the form nnnn.type_name.uavcan
 *
 *  DSDL{type_name} - An internal zero based ID for the transfer
 *                    format E.G DSDLNodeInfo would return the internal
 *                    constant index derived from the line that
 *                    UAVCAN_DSDL_TYPE_DEF(GetNodeInfo... is defined on
 *                    in uavcan_dsdl_defs.h (as of this writing it would
 *                     be the value 1)
 *
 *  DTID{type_name} - The uavcan Number nnn.name  E.G DTIDNodeInfo
 *
 *  ServiceNotMessage{type_name} - True for DTID that are uavcan Services Type
 *                                 Defined Transfer Type
 *
 *  SignatureCRC16{type_name}    - Required for multi-frame Transfer Type
 *
 *  PackedSize{type_name}        - The packed size of the Transfer Type
 *
 *  MailBox{type_name}           - When sending this Transfer Type, use this
 *                                 mailbox
 *
 *  Fifo{type_name}              - When receiving this Transfer Type, use this
 *                                 fifo. - Filers need to be configured
 *
 *  Fifo{type_name}              - The uavcan Number nnn.name  E.G DTIDNodeInfo
 *
 *  InTailInit{type_name}         - The value of the tail byte to expect at
 *                                  the end of a transfer
 *
 *  OutTailInit{type_name}        - The value of the tail byte to to use when
 *                                  sending this transfer
 *
 * For all UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length)
 * it defines the following enumeration constants:
 *
 * Mask{item name}                   - The mask that is positioned at the lsb and
 *                                     length long
 * BitPos{item name}                 - The bit position of the lsb
 * Length{item name}                 - The number of bits in the field
 * PayloadOffset{<name><field_name>} - The offset in the payload
 * PayloadLength{<name><field_name>} - The number of bytes
 *
 */


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/compiler.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "bitminip.h"
#include "can.h"
#include <systemlib/px4_macros.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define sizeof_member(t, m) sizeof(((t *)0)->m)
#define UAVCAN_STRLEN(x) sizeof((x))-1
#define uavcan_exclude(x, name) BITFEILD_EXCLUDE((x), BitPos##name,Length##name)
#define uavcan_make_uint16(d0, d1) (uint16_t)(((d1) << 8u) | (d0))

#define uavcan_dsdl_field(op, data_typ_name, field_name) (op##data_typ_name##field_name)
#define uavcan_bit_pos(data_typ_name, field_name)    uavcan_dsdl_field(BitPos, data_typ_name, field_name)
#define uavcan_bit_mask(data_typ_name, field_name)   uavcan_dsdl_field(Mask, data_typ_name, field_name)
#define uavcan_bit_count(data_typ_name, field_name) uavcan_dsdl_field(Length, data_typ_name, field_name)

#define uavcan_byte_offset(data_typ_name, field_name) uavcan_dsdl_field(PayloadOffset, data_typ_name, field_name)
#define uavcan_byte_count(data_typ_name, field_name) uavcan_dsdl_field(PayloadLength, data_typ_name, field_name)

#define uavcan_pack(d, data_typ_name, field_name) \
	(((d) << uavcan_bit_pos(data_typ_name, field_name)) & uavcan_bit_mask(data_typ_name, field_name))
#define uavcan_ppack(d, data_typ_name, field_name) uavcan_pack(d->field_name, data_typ_name, field_name)
#define uavcan_rpack(d, data_typ_name, field_name) uavcan_pack(d.field_name, data_typ_name, field_name)

#define uavcan_unpack(d, data_typ_name, field_name) \
	(((d) & uavcan_bit_mask(data_typ_name, field_name)) >> uavcan_bit_pos(data_typ_name, field_name))
#define uavcan_punpack(d, data_typ_name, field_name) uavcan_unpack(d->field_name, data_typ_name, field_name)
#define uavcan_runpack(d, data_typ_name, field_name) uavcan_unpack(d.field_name, data_typ_name, field_name)

#define uavcan_protocol_mask(field_name) (Mask##field_name)

/****************************************************************************
 * Auto Generated Public Type Definitions
 ****************************************************************************/
/* CAN ID fields */
#define UAVCAN_BIT_DEFINE(field_name, lsb_pos, length) Mask##field_name  =  BITFEILD_MASK((lsb_pos), (length)),
typedef enum uavcan_can_id_mask_t {
#include "uavcan_can_id_defs.h"
} uavcan_can_id_mask_t;
#undef UAVCAN_BIT_DEFINE

#define UAVCAN_BIT_DEFINE(field_name, lsb_pos, length) BitPos##field_name  =  (lsb_pos),
typedef enum uavcan_can_id_pos_t {
#include "uavcan_can_id_defs.h"
} uavcan_can_id_pos_t;
#undef UAVCAN_BIT_DEFINE

#define UAVCAN_BIT_DEFINE(field_name, lsb_pos, length) Length##field_name  =  (length),
typedef enum uavcan_can_id_length_t {
#include "uavcan_can_id_defs.h"
} uavcan_can_id_length_t;
#undef UAVCAN_BIT_DEFINE

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/
typedef enum uavcan_direction_t {
	InBound       = true,
	OutBound      = false,

	MessageIn     = InBound,
	MessageOut    = OutBound,
} uavcan_direction_t;

/*
 * Uavcan protocol CAN ID formats and Tail byte component definitions
 *
 * Most all the constants below are auto generated by the compiler in the
 * section above.
 * It defines Mask{item name}, BitPos{item name} and Length{item name}
 * for all UAVCAN_BIT_DEFINE({item name}, lsb_pos, length) entries
 *
 * [CAN ID[4]][data[0-7][tail[1]]]
 *
 */

/* General UAVCAN Constants */

typedef enum uavcan_general_t {
	UavcanPriorityMax = 0,
	UavcanPriorityMin = 31,

} uavcan_general_t;


/* CAN ID definitions for native data manipulation  */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
#pragma GCC diagnostic ignored "-Wpacked"
typedef begin_packed_struct struct can_id_t {
	union {
		uint32_t  u32;
		uint8_t   b[sizeof(uint32_t)];
	};
} end_packed_struct can_id_t;

/* UAVCAN CAN ID Usage: Message definition */

typedef struct uavcan_message_t {
uint32_t      source_node_id          : LengthUavCanMessageSourceNodeID;
uint32_t      service_not_message     : LengthUavCanMessageServiceNotMessage;
uint32_t      type_id                 : LengthUavCanMessageTypeID;
uint32_t      priority                : LengthUavCanMessagePriority;
} uavcan_message_t;

/* UAVCAN CAN ID Usage: Anonymous Message Constants */

typedef enum uavcan_anon_const_t {
	UavcanAnonymousNodeID  = 0,

} uavcan_anon_const_t;


/* UAVCAN CAN ID Usage: Anonymous Message definition */

typedef struct uavcan_anonymous_message_t {
uint32_t      source_node_id          : LengthUavCanAnonMessageSourceNodeID;
uint32_t      service_not_message     : LengthUavCanAnonMessageServiceNotMessage;
uint32_t      type_id                 : LengthUavCanAnonMessageTypeID;
uint32_t      discriminator           : LengthUavCanAnonMessageDiscriminator;
uint32_t      priority                : LengthUavCanAnonMessagePriority;
} uavcan_anonymous_message_t;

/* UAVCAN CAN ID Usage: Service Constants */

typedef enum uavcan_service_const_t {
	UavcanServiceRetries = 3,
	UavcanServiceTimeOutMs = 1000,

} uavcan_service_const_t;

/* UAVCAN CAN ID Usage: Service definition */

typedef struct uavcan_service_t {
uint32_t      source_node_id          : LengthUavCanServiceSourceNodeID;
uint32_t      service_not_message     : LengthUavCanServiceServiceNotMessage;
uint32_t      dest_node_id            : LengthUavCanServiceDestinationNodeID;
uint32_t      request_not_response    : LengthUavCanServiceRequestNotResponse;
uint32_t      type_id                 : LengthUavCanServiceTypeID;
uint32_t      priority                : LengthUavCanServicePriority;
} uavcan_service_t;

/* UAVCAN Tail Byte definitions for native data manipulation  */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
typedef begin_packed_struct struct can_tail_t {
	union {
		uint8_t   u8;
	};
} end_packed_struct can_tail_t;
#pragma GCC diagnostic pop

/* UAVCAN Tail Byte definitions */

typedef struct uavcan_tail_t {
uint8_t      transfer_id             : LengthUavCanTransferID;
uint8_t      toggle                  : LengthUavCanToggle;
uint8_t      eot                     : LengthUavCanEndOfTransfer;
uint8_t      sot                     : LengthUavCanStartOfTransfer;
} uavcan_tail_t;

/* UAVCAN Tail Byte Initialization constants */

typedef enum uavcan_tail_init_t {
	SingleFrameTailInit = (MaskUavCanStartOfTransfer | MaskUavCanEndOfTransfer),
	MultiFrameTailInit = (MaskUavCanStartOfTransfer),
	BadTailState = (MaskUavCanStartOfTransfer | MaskUavCanToggle),
	MaxUserPayloadLength = CanPayloadLength - sizeof(uavcan_tail_t),
} uavcan_tail_init_t;

/*
 * Assert that assumptions in code are true
 *  The code assumes it can manipulate a ALL sub protocol objects
 *  using MaskUavCanMessageServiceNotMessage, MaskUavCanMessagePriority
 *  and MaskUavCanMessageSourceNodeID
 */

CCASSERT(MaskUavCanServicePriority == MaskUavCanAnonMessagePriority);
CCASSERT(MaskUavCanServicePriority == MaskUavCanMessagePriority);
CCASSERT(MaskUavCanServiceSourceNodeID == MaskUavCanAnonMessageSourceNodeID);
CCASSERT(MaskUavCanServiceSourceNodeID == MaskUavCanMessageSourceNodeID);
CCASSERT(MaskUavCanMessageServiceNotMessage == MaskUavCanAnonMessageServiceNotMessage);
CCASSERT(MaskUavCanMessageServiceNotMessage == MaskUavCanMessageServiceNotMessage);

/****************************************************************************
 * Auto Generated Public Type Definitions
 ****************************************************************************/

/* UAVCAN DSDL Type Definitions*/
#define NA 0
#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length)

#define UAVCAN_DSDL_MESG_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	UAVCAN_DSDL_TYPE_DEF(Msg##name, (dtid), (signature), (packsize), (mailbox), (fifo), (inbound), (outbound))

#define UAVCAN_DSDL_SREQ_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	UAVCAN_DSDL_TYPE_DEF(Req##name, (dtid), (signature), (packsize), (mailbox), (fifo), (inbound), (outbound))

#define UAVCAN_DSDL_SRSP_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	UAVCAN_DSDL_TYPE_DEF(Rsp##name, (dtid), (signature), (packsize), (mailbox), (fifo), (inbound), (outbound))

#define END_COMPONENTS SizeDSDLComponents,
#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	DSDL##name,
typedef enum uavcan_dsdl_t {
#include "uavcan_dsdl_defs.h"
	SizeDSDL,
} uavcan_dsdl_t;
#undef UAVCAN_DSDL_TYPE_DEF
#undef END_COMPONENTS
#define END_COMPONENTS

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length)
#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	DTID##name  =  (dtid),
typedef enum uavcan_dsdl_dtid_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_dtid_t;
#undef UAVCAN_DSDL_TYPE_DEF

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	SignatureCRC16##name  =  (signature),
typedef enum uavcan_dsdl_sig_crc16_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_sig_crc16_t;
#undef UAVCAN_DSDL_TYPE_DEF

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	PackedSize##name  =  (packsize),
typedef enum uavcan_dsdl_packedsize_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_packedsize_t;
#undef UAVCAN_DSDL_TYPE_DEF

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	MailBox##name  =  (mailbox),
typedef enum uavcan_dsdl_mb_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_mb_t;
#undef UAVCAN_DSDL_TYPE_DEF

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	Fifo##name  =  (fifo),
typedef enum uavcan_dsdl_fifo_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_fifo_t;
#undef UAVCAN_DSDL_TYPE_DEF

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	InTailInit##name  =  (inbound),
typedef enum  uavcan_dsdl_inbound_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_inbound_t;
#undef UAVCAN_DSDL_TYPE_DEF

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound) \
	OutTailInit##name  =  (outbound),
typedef enum uavcan_dsdl_outbound_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_outbound_t;
#undef UAVCAN_DSDL_TYPE_DEF
#undef UAVCAN_DSDL_SRSP_DEF
#undef UAVCAN_DSDL_SREQ_DEF
#undef UAVCAN_DSDL_MESG_DEF
#undef UAVCAN_DSDL_BIT_DEF

/* UAVCAN DSDL Fields of Type Definitions definitions*/

#define UAVCAN_DSDL_TYPE_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound)
#define UAVCAN_DSDL_MESG_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound)
#define UAVCAN_DSDL_SREQ_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound)
#define UAVCAN_DSDL_SRSP_DEF(name, dtid, signature, packsize, mailbox, fifo, inbound, outbound)

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length) \
	Mask##data_typ_name##field_name  =  BITFEILD_MASK((lsb_pos), (length)),
typedef enum uavcan_dsdl_mask_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_mask_t;
#undef UAVCAN_DSDL_BIT_DEF

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length) \
	BitPos##data_typ_name##field_name  =  (lsb_pos),
typedef enum uavcan_dsdl_pos_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_pos_t;
#undef UAVCAN_DSDL_BIT_DEF

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length) \
	Length##data_typ_name##field_name  =  (length),
typedef enum uavcan_dsdl_length_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_length_t;
#undef UAVCAN_DSDL_BIT_DEF

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length) \
	PayloadOffset##data_typ_name##field_name  =  (payload_offset),
typedef enum uavcan_dsdl_payload_offset_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_payload_offset_t;
#undef UAVCAN_DSDL_BIT_DEF

#define UAVCAN_DSDL_BIT_DEF(data_typ_name, field_name, lsb_pos, length, payload_offset, payload_length) \
	PayloadLength##data_typ_name##field_name  =  (payload_length),
typedef enum uavcan_dsdl_payload_length_t {
#include "uavcan_dsdl_defs.h"
} uavcan_dsdl_payload_length_t;
#undef UAVCAN_DSDL_TYPE_DEF
#undef UAVCAN_DSDL_SRSP_DEF
#undef UAVCAN_DSDL_SREQ_DEF
#undef UAVCAN_DSDL_MESG_DEF
#undef UAVCAN_DSDL_BIT_DEF
#undef END_COMPONENTS
#undef NA

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Uavcan function return values */

typedef enum uavcan_error_t {
	UavcanOk            = 0,
	UavcanBootTimeout   = 1,
	UavcanError         = 3
} uavcan_error_t;


/*
 * Uavcan protocol CAN ID formats and Tail byte
 *
 * [CAN ID[4]][data[0-7][tail[1]]]
 *
 */

typedef begin_packed_struct struct uavcan_protocol_t {
	union {
		can_id_t  id;
		uavcan_message_t           msg;
		uavcan_anonymous_message_t ana;
		uavcan_service_t           ser;
	};
	union {
		can_tail_t     tail_init;
		uavcan_tail_t  tail;
	};
} end_packed_struct uavcan_protocol_t;


/*
 * Uavcan protocol DSDL Type Definitions
 */

/****************************************
 * Uavcan NodeStatus
 ****************************************/

typedef enum uavcan_NodeStatusConsts_t {
	MAX_BROADCASTING_PERIOD_MS = 1000,
	MIN_BROADCASTING_PERIOD_MS = 2,
	OFFLINE_TIMEOUT_MS = 2000,

	HEALTH_OK         = 0,
	HEALTH_WARNING    = 1,
	HEALTH_ERROR      = 2,
	HEALTH_CRITICAL   = 3,

	MODE_OPERATIONAL      = 0,
	MODE_INITIALIZATION   = 1,
	MODE_MAINTENANCE      = 2,
	MODE_SOFTWARE_UPDATE  = 3,

	MODE_OFFLINE          = 7,
} uavcan_NodeStatusConsts_t;

typedef begin_packed_struct struct uavcan_NodeStatus_t {
	uint32_t uptime_sec;
	union {
		uint8_t u8;
		struct {
uint8_t sub_mode: LengthNodeStatussub_mode;
uint8_t mode    : LengthNodeStatusmode;
uint8_t health  : LengthNodeStatushealth;
		};
	};
	uint16_t vendor_specific_status_code;
} end_packed_struct uavcan_NodeStatus_t;

/****************************************
 * Uavcan GetNodeInfo composition
 ****************************************/

/* SoftwareVersion */
typedef enum uavcan_SoftwareVersionConsts_t {
	OPTIONAL_FIELD_FLAG_VCS_COMMIT = 1,
	OPTIONAL_FIELD_FLAG_IMAGE_CRC  = 2,
} uavcan_SoftwareVersionConsts_t;

typedef begin_packed_struct struct uavcan_SoftwareVersion_t {
	uint8_t major;
	uint8_t minor;
	uint8_t optional_field_flags;
	uint32_t vcs_commit;
	uint64_t image_crc;
} end_packed_struct uavcan_SoftwareVersion_t;

CCASSERT(PackedSizeSoftwareVersion == sizeof(uavcan_SoftwareVersion_t));

/* HardwareVersion */

typedef begin_packed_struct struct uavcan_HardwareVersion_t {
	uint8_t major;
	uint8_t minor;
	uint8_t unique_id[PayloadLengthHardwareVersionunique_id];
	uint8_t certificate_of_authenticity_length;
	uint8_t certificate_of_authenticity[PayloadLengthHardwareVersioncertificate_of_authenticity];
} end_packed_struct uavcan_HardwareVersion_t;

typedef enum uavcan_HardwareVersionConsts_t {
	FixedSizeHardwareVersion = sizeof_member(uavcan_HardwareVersion_t, major) + \
				   sizeof_member(uavcan_HardwareVersion_t, minor) + \
				   sizeof_member(uavcan_HardwareVersion_t, unique_id) + \
				   sizeof_member(uavcan_HardwareVersion_t, certificate_of_authenticity_length),
} uavcan_HardwareVersionConsts_t;

typedef begin_packed_struct struct uavcan_GetNodeInfo_request_t {
	uint8_t empty[CanPayloadLength];
} end_packed_struct uavcan_GetNodeInfo_request_t;

/* GetNodeInfo Response */

typedef begin_packed_struct struct uavcan_GetNodeInfo_response_t {

	uavcan_NodeStatus_t nodes_status;;
	uavcan_SoftwareVersion_t software_version;
	uavcan_HardwareVersion_t hardware_version;

	uint8_t name[PayloadLengthGetNodeInfoname];
	uint8_t name_length;
} end_packed_struct uavcan_GetNodeInfo_response_t;

typedef enum uavcan_GetNodeInfoConsts_t {
	FixedSizeGetNodeInfo = PackedSizeMsgNodeStatus + PackedSizeSoftwareVersion +  FixedSizeHardwareVersion,

} uavcan_GetNodeInfoConsts_t;

/****************************************
 * Uavcan LogMessage
 ****************************************/

typedef enum uavcan_LogMessageConsts_t {
	LOGMESSAGE_LEVELDEBUG                 = 0,
	LOGMESSAGE_LEVELINFO                  = 1,
	LOGMESSAGE_LEVELWARNING               = 2,
	LOGMESSAGE_LEVELERROR                 = 3,
} uavcan_LogMessageConsts_t;

typedef begin_packed_struct struct uavcan_LogMessage_t {
	uint8_t level;
	uint8_t source[uavcan_byte_count(LogMessage, source)];
	uint8_t text[uavcan_byte_count(LogMessage, text)];
} end_packed_struct uavcan_LogMessage_t;

CCASSERT(sizeof(uavcan_LogMessage_t) == PackedSizeMsgLogMessage);

/****************************************
 * Uavcan Allocation
 ****************************************/

typedef enum uavcan_AllocationConsts_t {
	MAX_REQUEST_PERIOD_MS                 = 1000,
	MIN_REQUEST_PERIOD_MS                 = 50,
	MAX_FOLLOWUP_DELAY_MS                 = 400,
	MIN_FOLLOWUP_DELAY_MS                 = 0,
	FOLLOWUP_TIMEOUT_MS                   = 500,
	MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST    = 6,
	ANY_NODE_ID                           = 0,
	PriorityAllocation                    = UavcanPriorityMin - 1,
} uavcan_AllocationConsts_t;

typedef begin_packed_struct struct uavcan_Allocation_t {
	uint8_t node_id; /* bottom bit is the first part flag */
	uint8_t unique_id[PayloadLengthAllocationunique_id];
} end_packed_struct uavcan_Allocation_t;


/****************************************
 * Uavcan Path
 ****************************************/


typedef begin_packed_struct struct uavcan_Path_t {
	uint8_t u8[PayloadLengthPathpath];
} uavcan_Path_t;

typedef enum uavcan_PathConst_t {
	SEPARATOR                 = '/',
} end_packed_struct uavcan_PathConst_t;

/****************************************
 * Uavcan GetInfo Composition
 ****************************************/

typedef enum uavcan_ErrorConst_t {
	FILE_ERROR_OK                = 0,
	FILE_ERROR_UNKNOWN_ERROR     = 32767,
	FILE_ERROR_NOT_FOUND         = 2,
	FILE_ERROR_IO_ERROR          = 5,
	FILE_ERROR_ACCESS_DENIED     = 13,
	FILE_ERROR_IS_DIRECTORY      = 21,
	FILE_ERROR_INVALID_VALUE     = 22,
	FILE_ERROR_FILE_TOO_LARGE    = 27,
	FILE_ERROR_OUT_OF_SPACE      = 28,
	FILE_ERROR_NOT_IMPLEMENTED   = 38,
} uavcan_ErrorConst_t;

typedef begin_packed_struct struct uavcan_Error_t {
	uint16_t value;
} end_packed_struct uavcan_Error_t;

typedef enum uavcan_EntryTypeConst_t {
	ENTRY_TYPE_FLAG_FILE      = 1,
	ENTRY_TYPE_FLAG_DIRECTORY = 2,
	ENTRY_TYPE_FLAG_SYMLINK   = 4,
	ENTRY_TYPE_FLAG_READABLE  = 8,
	ENTRY_TYPE_FLAG_WRITEABLE = 16,
} uavcan_EntryTypeConst_t;

typedef begin_packed_struct struct uavcan_EntryType_t {
	uint8_t flags;
} end_packed_struct uavcan_EntryType_t;


/****************************************
 * Uavcan BeginFirmwareUpdate
 ****************************************/

typedef enum uavcan_BeginFirmwareUpdateConst_t {
	ERROR_OK               = 0,
	ERROR_INVALID_MODE     = 1,
	ERROR_IN_PROGRESS      = 2,
	ERROR_UNKNOWN          = 255,
} uavcan_BeginFirmwareUpdateConst_t;

typedef begin_packed_struct struct uavcan_BeginFirmwareUpdate_request {
	uint8_t source_node_id;
	uavcan_Path_t image_file_remote_path;
} end_packed_struct uavcan_BeginFirmwareUpdate_request;

typedef begin_packed_struct struct uavcan_BeginFirmwareUpdate_response {
	uint8_t error;
} end_packed_struct uavcan_BeginFirmwareUpdate_response;



/****************************************
 * Uavcan GetInfo
 ****************************************/

typedef begin_packed_struct struct uavcan_GetInfo_request_t {
	uavcan_Path_t path;
} uavcan_GetInfo_request_t;
typedef enum uavcan_GetInfo_requestConst_t {
	FixedSizeGetInfoRequest = 0,

} end_packed_struct uavcan_GetInfo_requestConst_t;

typedef begin_packed_struct struct uavcan_GetInfo_response_t {
	uint32_t                size;
	uint8_t                 msbsize;
	uavcan_Error_t          error;
	uavcan_EntryType_t      entry_type;
} end_packed_struct uavcan_GetInfo_response_t;


/****************************************
 * Uavcan Read Composition
 ****************************************/

typedef begin_packed_struct struct uavcan_Read_request_t {
	uint32_t offset;
	uint8_t msboffset;
	uavcan_Path_t path;
} end_packed_struct uavcan_Read_request_t;


typedef enum uavcan_ReadRequestConsts_t {
	FixedSizeReadRequest            = sizeof_member(uavcan_Read_request_t, offset) + \
					  sizeof_member(uavcan_Read_request_t, msboffset),
} uavcan_ReadRequestConsts_t;


typedef begin_packed_struct struct uavcan_Read_response_t {
	uavcan_Error_t error;
	uint8_t data[PayloadLengthReaddata];
} end_packed_struct uavcan_Read_response_t;

/****************************************************************************
 * Global Variables
 ****************************************************************************/
extern uint8_t g_this_node_id;
extern uint8_t g_server_node_id;
extern uint8_t g_uavcan_priority;
/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: uavcan_pack_GetNodeInfo_response
 *
 * Description:
 *   This function packs the data of a uavcan_NodeStatus_t into
 *   a uavcan_GetNodeInfo_response_t structure as array of bytes.
 *   Then it packs the uavcan_GetNodeInfo_response_t
 *
 * Input Parameters:
 *   response   The uavcan_GetNodeInfo_response_t to be packed
 *   node_status - The uavcan_NodeStatus_t that is part of the composition
 *
 * Returned value:
 *   Number of bytes written.
 *
 ****************************************************************************/

size_t uavcan_pack_GetNodeInfo_response(uavcan_GetNodeInfo_response_t
					*response);

/****************************************************************************
 * Name: uavcan_tx_dsdl
 *
 * Description:
 *   This helper function sends a uavcan service protocol, it
 *   assumes the protocol object has the destination node id set.
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the send,
 *                the transfer with the dest_node_id set to that of the
 *                node we are making the request of.
 *   transfer    - A pointer to the packed data of the transfer to be sent.
 *   length     - The number of bytes of data
 *
* Returned value:
 *   The UavcanOk of the data sent. Anything else indicates if a timeout
 *   occurred.
 *
 ****************************************************************************/

uavcan_error_t uavcan_tx_dsdl(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
			      const uint8_t *transfer, size_t transfer_length);

/****************************************************************************
 * Name: uavcan_rx_dsdl
 *
 * Description:
 *   This function receives a uavcan Service response protocol transfer
 *
 * Input Parameters:
 *   dsdl       - An Uavcan DSDL Identifier (Auto Generated)
 *   protocol   - A pointer to a uavcan_protocol_t to configure the receive,
 *                based the dsdl for the DTID Service.
 *                If the request must come from a specific server
 *                then protocol->ser.source_node_id, should be set
 *                to that node id;
 *
 *   in_out_transfer_length - The number of bytes of data to receive and the
 *                            number received.
 *   timeout_ms - The amount of time in mS to wait for the initial transfer
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/
uavcan_error_t uavcan_rx_dsdl(uavcan_dsdl_t dsdl, uavcan_protocol_t *protocol,
			      uint8_t *transfer, size_t *in_out_transfer_length,
			      uint32_t timeout_ms);

/****************************************************************************
 * Name: uavcan_tx_log_message
 *
 * Description:
 *   This functions sends uavcan LogMessage type data. The Source will be
 *   taken from the application defined debug_log_source
 *
 * Input Parameters:
 *   level   - Log Level of the LogMessage Constants DEBUG, INFO, WARN, ERROR
 *   stage   - The Stage the application is at. see Aplication defined
 *             LOGMESSAGE_STAGE_x
 *   status  - The status of that stage. See Application defined
 *             LOGMESSAGE_RESULT_x
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
/* The application must define this */
extern const uint8_t debug_log_source[uavcan_byte_count(LogMessage, source)];

void uavcan_tx_log_message(uavcan_LogMessageConsts_t level, uint8_t stage,
			   uint8_t status);

/****************************************************************************
 * Name: uavcan_tx_allocation_message
 *
 * Description:
 *   This function sends a uavcan allocation message.
 *
 * Input Parameters:
 *   requested_node_id - This node's preferred node id 0 for no preference.
 *   unique_id_length  - This node's length of it's unique identifier.
 *   unique_id         - A pointer to the bytes that represent unique
 *                       identifier.
 *   unique_id_offset  - The offset equal 0 or the number of bytes in the
 *                       the last received message that matched the unique ID
 *                       field.
 *   random             - The value to use as the discriminator of the
 *                        anonymous message
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

void uavcan_tx_allocation_message(uint8_t requested_node_id,
				  size_t unique_id_length,
				  const uint8_t *unique_id,
				  uint8_t unique_id_offset,
				  uint16_t random);
