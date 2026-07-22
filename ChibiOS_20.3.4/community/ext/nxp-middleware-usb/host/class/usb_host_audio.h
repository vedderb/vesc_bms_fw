/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016, 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_HOST_AUDIO_H__
#define __USB_HOST_AUDIO_H__

/*******************************************************************************
 * Audio class private structure, enumerations, macros
 ******************************************************************************/
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Structure for an AUDIO class descriptor according to the 6.2.1 in Audio 1.0 specification*/

/* Audio 1.0 class codes */
#define SET_COMMAND (0x00)
#define GET_COMMAND (0x80)
#define CUR_REQUEST (0x01)
#define MIN_REQUEST (0x02)
#define MAX_REQUEST (0x03)
#define RES_REQUEST (0x04)
#define MEM_REQUEST (0x05)
#define GET_STATUS  (0xFF)

#define ITF_REQUEST (0x21)
#define EP_REQUEST  (0x22)

#define AUDIO_FU_MUTE_MASK       0x01
#define AUDIO_FU_VOLUME_MASK     0x02
#define AUDIO_FU_BASS_MASK       0x04
#define AUDIO_FU_MID_MASK        0x08
#define AUDIO_FU_TREBLE_MASK     0x10
#define AUDIO_FU_GRAPHIC_EQ_MASK 0x20
#define AUDIO_FU_AGC_MASK        0x40
#define AUDIO_FU_DELAY_MASK      0x80
#define AUDIO_FU_BASS_BOOST_MASK 0x01

/* USB audio1.0 Endpoint Control Selectors */
#define AUDIO_EP_CONTROL_UNDEFINED  (0x00)
#define AUDIO_SAMPLING_FREQ_CONTROL (0x01)
#define AUDIO_PITCH_CONTROL         (0x02)
/* USB audio2.0 Endpoint Control Selectors */
#define AUDIO_EP_CONTROL_UNDEFINED_20     (0x00)
#define AUDIO_EP_PITCH_CONTROL_20         (0x01)
#define AUDIO_EP_DATA_OVERRUN_CONTRO_20   (0x02)
#define AUDIO_EP_DATA_UNDERRUN_CONTROL_20 (0x03)

/* USB audio1.0 MASK*/
#define AUDIO_SAMPLING_FREQ_MASK (0x01)
#define AUDIO_PITCH_MASK         (0x02)
typedef enum _fu_request_code
{
    kUSB_AudioCurMute = 0,
    kUSB_AudioCurVolume,
    kUSB_AudioMinVolume,
    kUSB_AudioMaxVolume,
    kUSB_AudioResVolume,
    NUMBER_OF_FEATURE_COMMANDS,
} fu_request_code_t;

typedef enum _ep_request_code
{
    kUSB_AudioCurPitch = 0,
    kUSB_AudioCurSamplingFreq,
    kUSB_AudioMinSamplingFreq,
    kUSB_AudioMaxSamplingFreq,
    kUSB_AudioResSamplingFreq,
    NUMBER_OF_ENDPOINT_COMMANDS,
} ep_request_code_t;

typedef union _audio_descriptor_union
{
    uint8_t *bufr;
    usb_descriptor_common_t *common;
    usb_descriptor_device_t *device;
    usb_descriptor_configuration_t *configuration;
    usb_descriptor_interface_t *interface;
    usb_descriptor_endpoint_t *endpoint;
} audio_descriptor_union_t;

/* Audio command structure */
typedef struct _usb_audio_request
{
    uint8_t controlMask;
    uint8_t typeRequest;
    uint8_t codeRequest;
    uint8_t requestValue;
    uint8_t length;
} usb_audio_request_t;

/*******************************************************************************
 * Audio class public structure, enumeration, macros, functions
 ******************************************************************************/
/*!
 * @addtogroup usb_host_audio_drv
 * @{
 */
/*! @brief Audio class code */
#define USB_AUDIO_CLASS_CODE 1
/*! @brief Audio class  control interface code*/
#define USB_AUDIO_SUBCLASS_CODE_CONTROL 1
/*! @brief Audio class  stream interface code*/
#define USB_AUDIO_SUBCLASS_CODE_AUDIOSTREAMING 2

/* Audio Device Class Specification Release Number in Binary-Coded Decimal.*/
#define AUDIO_DEVICE_VERSION_01 0x0100U
#define AUDIO_DEVICE_VERSION_02 0x0200U

/*! @brief AUDIO class-specific feature unit get current mute command*/
#define USB_AUDIO_GET_CUR_MUTE 0x80
/*! @brief AUDIO class-specific feature unit set current mute command*/
#define USB_AUDIO_SET_CUR_MUTE 0x00
/*! @brief AUDIO class-specific feature unit get current volume command*/
#define USB_AUDIO_GET_CUR_VOLUME 0x81
/*! @brief AUDIO class-specific feature unit set current volume command*/
#define USB_AUDIO_SET_CUR_VOLUME 0x01
/*! @brief AUDIO class-specific feature unit get minimum volume command*/
#define USB_AUDIO_GET_MIN_VOLUME 0x82
/*! @brief AUDIO class-specific feature unit set minimum volume command*/
#define USB_AUDIO_SET_MIN_VOLUME 0x02
/*! @brief AUDIO class-specific feature unit get maximum volume command*/
#define USB_AUDIO_GET_MAX_VOLUME 0x83
/*! @brief AUDIO class-specific feature unit set maximum volume command*/
#define USB_AUDIO_SET_MAX_VOLUME 0x03
/*! @brief AUDIO class-specific feature unit get resolution volume command*/
#define USB_AUDIO_GET_RES_VOLUME 0x84
/*! @brief AUDIO class-specific feature unit set resolution volume command*/
#define USB_AUDIO_SET_RES_VOLUME 0x04

/*! @brief AUDIO class-specific endpoint get current pitch control command*/
#define USB_AUDIO_GET_CUR_PITCH 0x80
/*! @brief AUDIO class-specific endpoint set current pitch control command*/
#define USB_AUDIO_SET_CUR_PITCH 0x00
/*! @brief AUDIO class-specific endpoint get current sampling frequency command*/
#define USB_AUDIO_GET_CUR_SAMPLING_FREQ 0x81
/*! @brief AUDIO class-specific endpoint set current sampling frequency command*/
#define USB_AUDIO_SET_CUR_SAMPLING_FREQ 0x01
/*! @brief AUDIO class-specific endpoint get minimum sampling frequency command*/
#define USB_AUDIO_GET_MIN_SAMPLING_FREQ 0x82
/*! @brief AUDIO class-specific endpoint set minimum sampling frequency command*/
#define USB_AUDIO_SET_MIN_SAMPLING_FREQ 0x02
/*! @brief AUDIO class-specific endpoint get maximum sampling frequency command*/
#define USB_AUDIO_GET_MAX_SAMPLING_FREQ 0x83
/*! @brief AUDIO class-specific endpoint set maximum sampling frequency command*/
#define USB_AUDIO_SET_MAX_SAMPLING_FREQ 0x03
/*! @brief AUDIO class-specific endpoint get resolution sampling frequency command*/
#define USB_AUDIO_GET_RES_SAMPLING_FREQ 0x84
/*! @brief AUDIO class-specific endpoint set resolution sampling frequency command*/
#define USB_AUDIO_SET_RES_SAMPLING_FREQ 0x04

/*Audio Class-Specific AC Interface Descriptor Subtypes*/
#define USB_AUDIO_DESC_SUBTYPE_AUDIO_UNDEFINED 0x00U /*undefined Descriptor*/
/*remain two input terminal descriptor macro for compatibility, please use the latter one*/
#define USB_DESC_SUBTYPE_AUDIO_CS_HEADER 0x01U
#define USB_AUDIO_DESC_SUBTYPE_CS_HEADER 0x01U /*Header Descriptor*/
/*remain two input terminal descriptor macro for compatibility, please use the latter one*/
#define USB_DESC_SUBTYPE_AUDIO_CS_IT    0x02U /*Input Terminal Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_INPUT 0x02U /*Input Terminal Descriptor*/

/*remain two output terminal descriptor macro for compatibility, please use the latter one*/
#define USB_DESC_SUBTYPE_AUDIO_CS_OT     0x03U /*Output Terminal Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_OUTPUT 0x03U /*Output Terminal Descriptor*/

#define USB_AUDIO_DESC_SUBTYPE_CS_MIXER    0x04U /*Mixer Unit Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_SELECTOR 0x05U /*Selector Unit Descriptor*/
/*remain two feature uint descriptor macro for compatibility, please use the latter one*/
#define USB_DESC_SUBTYPE_AUDIO_CS_FU      0x06U
#define USB_AUDIO_DESC_SUBTYPE_CS_FEATURE 0x06U /*Feature Unit Descriptor*/

#define USB_AUDIO_DESC_SUBTYPE_CS_EFEET      0x07U /*Effect Unit Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_PROCESSING 0x08U /*Processing Unit Descriptor*/
/*the following descriptor is not support by Audio version 01*/
#define USB_AUDIO_DESC_SUBTYPE_CS_EXTENSION      0x09U /*Extension Unit Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_CLOCK_SOURE    0x0AU /*Clock Source Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_CLCOK_SELECTOR 0x0BU /*Clock Selector Descriptor*/
#define USB_AUDIO_DESC_SUBTYPE_CS_CLOCK_MULTIPLE 0x0CU /*Clock Multiple Descriptor*/

/*remain two endpoint macro for compatibility, please use the latter one*/
#define USB_DESC_CLASS_ENDPOINT_GENERAL       0x01U
#define USB_AUDIO_DESC_CLASS_ENDPOINT_GENERAL 0x01U

/*! @brief Audio device class-specific descriptor type */
#define USB_AUDIO_DESCRIPTOR_TYPE_CS_DEVICE        (0x21U)
#define USB_AUDIO_DESCRIPTOR_TYPE_CS_CONFIGURATION (0x22U)
#define USB_AUDIO_DESCRIPTOR_TYPE_CS_STRING        (0x23U)
#define USB_AUDIO_DESCRIPTOR_TYPE_CS_INTERFACE     (0x24U)
#define USB_AUDIO_DESCRIPTOR_TYPE_CS_ENDPOINT      (0x25U)

/*Feature Unit Control Selectors, audio1.0 and audio2.0 can share same featre unit control selector code*/
#define AUDIO_FU_MUTE       0x01U
#define AUDIO_FU_VOLUME     0x02U
#define AUDIO_FU_BASS       0x03U
#define AUDIO_FU_MID        0x04U
#define AUDIO_FU_TREBLE     0x05U
#define AUDIO_FU_GRAPHIC_EQ 0x06U
#define AUDIO_FU_AGC        0x07U
#define AUDIO_FU_DELAY      0x08U
#define AUDIO_FU_BASS_BOOST 0x09U

/*Audio Class-Specific AS Interface Descriptor Subtypes*/
/*audio 1.0*/
/*remain two subtype macro for compatibility, please use the latter one*/
#define USB_DESC_SUBTYPE_AS_CS_GENERAL     0X01U
#define USB_DESC_SUBTYPE_AS_CS_FORMAT_TYPE 0X02U

#define USB_AUDIO_DESC_SUBTYPE_AS_GENERAL              0x01U
#define USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_TYPE          0x02U
#define USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_SEPCIFIC_TYPE 0x03U

/*audio 2.0*/
#define USB_AUDIO_DESC_SUBTYPE_AS_GENERAL_20        0x01U
#define USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_TYPE_20    0x02U
#define USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_ENCODER_20 0x03U
#define USB_AUDIO_DESC_SUBTYPE_AS_FORMAT_DECODER_20 0x04U

/*Audio1.0  Class-Specific Request Codes*/
#define USB_AUDIO_CS_REQUEST_CODE_SET_CUR  0x01U
#define USB_AUDIO_CS_REQUEST_CODE_GET_CUR  0x81U
#define USB_AUDIO_CS_REQUEST_CODE_SET_MIN  0x02U
#define USB_AUDIO_CS_REQUEST_CODE_GET_MIN  0x82U
#define USB_AUDIO_CS_REQUEST_CODE_SET_MAX  0x03U
#define USB_AUDIO_CS_REQUEST_CODE_GET_MAX  0x83U
#define USB_AUDIO_CS_REQUEST_CODE_SET_RES  0x04U
#define USB_AUDIO_CS_REQUEST_CODE_GET_RES  0x84U
#define USB_AUDIO_CS_REQUEST_CODE_SET_MEM  0x05U
#define USB_AUDIO_CS_REQUEST_CODE_GET_MEN  0x85U
#define USB_AUDIO_CS_REQUEST_CODE_GET_STAT 0xFFU

/*Audio2.0  Class-Specific Request Codes*/
#define USB_AUDIO_CS_REQUEST_CODE_CUR_20   0x01U
#define USB_AUDIO_CS_REQUEST_CODE_RANGE_20 0x02U

/*Audio2.0 Clock Source Control Selectors Codes*/
#define USB_AUDIO_CS_SAM_FREQ_CONTROL_20    0x01U
#define USB_AUDIO_CS_CLOCK_VALID_CONTROL_20 0x02U

/*common audio endpoint type code
00 = Data endpoint
01 = Feedback endpoint
10 = Implicit feedback Data endpoit, usb Spec 9.6*/
#define USB_AUDIO_EP_TYPE_DATA     0x00U
#define USB_AUDIO_EP_TYPE_FEEDBACK 0x01U
#define USB_AUDIO_EP_TYPE_IMPLICIT 0x02U

/*endpoint control selectors*/
/*audio 1.0*/
#define USB_AUDIO_EP_CS_SAMPING_FREQ_CONTROL 0x01U
#define USB_AUDIO_EP_CS_PINTCH_CONTROL       0x02U
/*audio 2.0*/
#define USB_AUDIO_EP_CS_PINTCH_CONTROL_20   0x01U
#define USB_AUDIO_EP_CS_OVERRUN_CONTROL_20  0x02U
#define USB_AUDIO_EP_CS_UNDERRUN_CONTROL_20 0x03U

struct _usb_audio_2_0_control_range_layout3_struct
{
    uint8_t wNumSubRanges[2];
    uint8_t wMIN[4];
    uint8_t wMAX[4];
    uint8_t wRES[4];
};
typedef struct _usb_audio_2_0_control_range_layout3_struct usb_audio_2_0_control_range_layout3_struct_t;

struct _usb_audio_2_0_control_range_layout2_struct
{
    uint8_t wNumSubRanges[2];
    uint8_t wMIN[2];
    uint8_t wMAX[2];
    uint8_t wRES[2];
};
typedef struct _usb_audio_2_0_control_range_layout2_struct usb_audio_2_0_control_range_layout2_struct_t;

/*! @brief Audio control interface header descriptor structure */
typedef struct _usb_audio_2_0_ctrl_header_desc
{
    uint8_t blength;            /*!< Total size of the header descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio header descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of an audio header descriptor*/
    uint8_t bcdcdc[2];          /*!< Audio Device Class Specification Release Number in Binary-Coded Decimal*/
    uint8_t bCategory;          /*!< Constant, indicating the primary use of this audio function, as intended by the
                                   manufacturer. See Appendix A.7, Audio Function Category Codes.*/
    uint8_t wtotallength[2];    /*!< Total number of bytes returned for the class-specific AudioControl interface
                                   descriptor. Includes the combined length of this descriptor header and all unit and
                                   terminal descriptors.*/
    uint8_t bmControls;         /*!< D1..0: Latency Control
                                     D7..2: Reserved. Must be set to 0.*/
} usb_audio_2_0_ctrl_header_desc_t;

/*! @brief Audio control interface clock source descriptor structure */
typedef struct _usb_audio_2_0_ctrl_clock_source_desc
{
    uint8_t blength;            /*!< Total size of the header descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio header descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of an audio header descriptor*/
    uint8_t bClockID; /*!<Constant uniquely identifying the Clock Source Entity within the audio function. This value is
                         used in all requests to address this Entity.*/
    uint8_t bmAttributes;   /*!< D1..0: Clock Type, D2: Clock synchronized to SOF*/
    uint8_t bmControls;     /*!< D1..0: Clock Frequency D3..2: Clock Validity Control.*/
    uint8_t bAssocTerminal; /*!< Terminal ID of the Terminal that is associated with this Clock Source.*/
    uint8_t iClockSource;   /*!<Index of a string descriptor, describing the Clock Source Entity.*/

} usb_audio_2_0_ctrl_clock_source_desc_t;

/*! @brief Audio control interface input terminal descriptor structure */
typedef struct _usb_audio_2_0_ctrl_it_desc
{
    uint8_t blength;            /*!< Total size of the input terminal descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio input terminal descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of audio input terminal descriptor*/
    uint8_t bterminalid; /*!< Constant uniquely identifying the Terminal within the audio function. This value is used
                            in all requests to address this Terminal*/
    uint8_t wterminaltype[2];   /*!< Constant characterizing the type of Terminal*/
    uint8_t bassocterminal;     /*!< ID of the Output Terminal to which this Input Terminal is associated*/
    uint8_t bCSourceID;         /*!< ID of the Clock Entity to which this Input Terminal is connected.*/
    uint8_t bNrChannels;        /*!< Describes the spatial location of the logical channels.*/
    uint8_t bmChannelConfig[4]; /*!< Describes the spatial location of the logical channels.*/
    uint8_t iChannelNames;      /*!< Index of a string descriptor, describing the Input Terminal*/
    uint8_t bmControls[2];      /*!<D1..0: Copy Protect Control
                                    D3..2: Connector Control
                                    D5..4: Overload Control
                                    D7..6: Cluster Control
                                    D9..8: Underflow Control
                                    D11..10: Overflow Control
                                    D15..12: Reserved. Must be set to 0.*/
    uint8_t iTerminal;          /*!< Index of a string descriptor, describing the Input Terminal.*/
} usb_audio_2_0_ctrl_it_desc_t;

/*! @brief Audio control interface output terminal descriptor structure */
typedef struct _usb_audio_2_0_ctrl_ot_desc
{
    uint8_t blength;            /*!< Total size of the output terminal descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio output terminal descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of audio output terminal descriptor*/
    uint8_t bterminalid; /*!< Constant uniquely identifying the Terminal within the audio function. This value is used
                            in all requests to address this Terminal*/
    uint8_t wterminaltype[2]; /*!< Constant characterizing the type of Terminal*/
    uint8_t
        bassocterminal;    /*!< IConstant, identifying the Input Terminal to which this Output Terminal is associated.*/
    uint8_t bSourceID;     /*!< ID of the Unit or Terminal to which this Terminal is connected.*/
    uint8_t bCSourceID;    /*!< ID of the Clock Entity to which this Output Terminal is connected.*/
    uint8_t bmControls[2]; /*!<D1..0: Copy Protect Control
                               D3..2: Connector Control
                               D5..4: Overload Control
                               D7..6: Underflow Control
                               D9..8: Overflow Control
                               D15..10: Reserved. Must be set to 0.*/
    uint8_t iTerminal;     /*!< Index of a string descriptor, describing the Input Terminal.*/
} usb_audio_2_0_ctrl_ot_desc_t;

/*! @brief Audio control interface feature unit descriptor structure */
typedef struct _usb_audio_2_0_ctrl_fu_desc
{
    uint8_t blength;            /*!< Total size of the output terminal descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio output terminal descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of audio output terminal descriptor*/
    uint8_t bunitid;   /*!< Constant uniquely identifying the unit within the audio function. This value is used in all
                          requests to address this unit*/
    uint8_t bsourceid; /*!< ID of the Unit or Terminal to which this Feature Unit is connected*/
    uint8_t bmaControls0[4]; /*!< The Controls bitmap for master channel 0:*/
} usb_audio_2_0_ctrl_fu_desc_t;

/*! @brief Audio as isochronous audio data endpoint descriptor structure */
typedef struct _usb_audio_2_0_stream_specific_iso_endp_desc
{
    uint8_t blength;            /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of the descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of the descriptor*/
    uint8_t bmattributes;   /*!< A bit in the range D6..0 set to 1 indicates that the mentioned Control is supported by
                               this endpoint*/
    uint8_t bmControls;     /*!< D1..0: Pitch Control
                                 D3..2: Data Overrun Control
                                 D5..4: Data Underrun Control
                                 D7..6: Reserved. Must be set to 0. */
    uint8_t blockdlayunits; /*!< Indicates the units used for the wLockDelay field*/
    uint8_t wlockdelay[2];  /*!< Indicates the time it takes this endpoint to reliably lock its internal clock recovery
                               circuitry. Units used depend on the value of the bLockDelayUnits field.*/
} usb_audio_2_0_stream_specific_iso_endp_desc_t;

typedef struct _usb_audio_ctrl_common_header_desc
{
    uint8_t blength;            /*!< Total size of the header descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio header descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of an audio header descriptor*/
    uint8_t bcdcdc[2];          /*!< Audio Device Class Specification Release Number in Binary-Coded Decimal*/
} usb_audio_ctrl_common_header_desc_t;
/*! @brief Audio control interface header descriptor structure */
typedef struct _usb_audio_ctrl_header_desc
{
    uint8_t blength;            /*!< Total size of the header descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio header descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of an audio header descriptor*/
    uint8_t bcdcdc[2];          /*!< Audio Device Class Specification Release Number in Binary-Coded Decimal*/
    uint8_t wtotallength[2];    /*!< Total number of bytes returned for the class-specific AudioControl interface
                                   descriptor. Includes the combined length of this descriptor header and all unit and
                                   terminal descriptors.*/
    uint8_t bincollection;      /*!< The number of AudioStreaming and MIDIStreaming interfaces in the Audio Interface
                                   Collection to which this AudioControl interface belongs to*/
} usb_audio_ctrl_header_desc_t;

/*! @brief Audio control interface input terminal descriptor structure */
typedef struct _usb_audio_ctrl_it_desc
{
    uint8_t blength;            /*!< Total size of the input terminal descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio input terminal descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of audio input terminal descriptor*/
    uint8_t bterminalid; /*!< Constant uniquely identifying the Terminal within the audio function. This value is used
                            in all requests to address this Terminal*/
    uint8_t wterminaltype[2];  /*!< Constant characterizing the type of Terminal*/
    uint8_t bassocterminal;    /*!< ID of the Output Terminal to which this Input Terminal is associated*/
    uint8_t bnrchannels;       /*!< Number of logical output channels in the Terminal's output audio channel cluster*/
    uint8_t wchannelconfig[2]; /*!< Describes the spatial location of the logical channels.*/
    uint8_t ichannelnames;     /*!< Index of a string descriptor, describing the name of the first logical channel*/
    uint8_t iterminal;         /*!<Index of a string descriptor, describing the Input Terminal*/
} usb_audio_ctrl_it_desc_t;

/*! @brief Audio control interface output terminal descriptor structure */
typedef struct _usb_audio_ctrl_ot_desc
{
    uint8_t blength;            /*!< Total size of the output terminal descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio output terminal descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of audio output terminal descriptor*/
    uint8_t bterminalid; /*!< Constant uniquely identifying the Terminal within the audio function. This value is used
                            in all requests to address this Terminal*/
    uint8_t wterminaltype[2]; /*!< Constant characterizing the type of Terminal*/
    uint8_t bassocterminal; /*!< Constant, identifying the Input Terminal to which this Output Terminal is associated*/
    uint8_t bsourceid;      /*!< ID of the Unit or Terminal to which this Terminal is connected*/
    uint8_t iterminal;      /*!< Index of a string descriptor, describing the Output Terminal*/
} usb_audio_ctrl_ot_desc_t;

/*! @brief Audio control interface feature unit descriptor structure */
typedef struct _usb_audio_ctrl_fu_desc
{
    uint8_t blength;            /*!< Total size of the output terminal descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of audio output terminal descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of audio output terminal descriptor*/
    uint8_t bunitid;   /*!< Constant uniquely identifying the unit within the audio function. This value is used in all
                          requests to address this unit*/
    uint8_t bsourceid; /*!< ID of the Unit or Terminal to which this Feature Unit is connected*/
    uint8_t bcontrolsize; /*!< Size in bytes of an element of the bmaControls*/
} usb_audio_ctrl_fu_desc_t;

/*! @brief Audio as isochronous audio data endpoint descriptor structure */
typedef struct _usb_audio_stream_specific_iso_endp_desc
{
    uint8_t blength;            /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of the descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of the descriptor*/
    uint8_t bmattributes;   /*!< A bit in the range D6..0 set to 1 indicates that the mentioned Control is supported by
                               this endpoint*/
    uint8_t blockdlayunits; /*!< Indicates the units used for the wLockDelay field*/
    uint8_t wlockdelay[2];  /*!< Indicates the time it takes this endpoint to reliably lock its internal clock recovery
                               circuitry. Units used depend on the value of the bLockDelayUnits field.*/
} usb_audio_stream_specific_iso_endp_desc_t;

/*! @brief Audio standard as isochronous synch endpoint descriptor structure */
typedef struct _usb_audio_stream_synch_endp_desc
{
    uint8_t blength;           /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;   /*!< Descriptor type of the endpoint descriptor*/
    uint8_t bendpointaddress;  /*!< The address of the endpoint on the USB device described by this descriptor*/
    uint8_t bmattributes;      /*!< D3..2: Synchronization type, D1..0: Transfer type*/
    uint8_t wmaxpacketsize[2]; /*!< Maximum packet size this endpoint is capable of sending or receiving when this
                                  configuration is selected*/
    uint8_t binterval;         /*!< Interval for polling endpoint for data transfers expressed in milliseconds*/
    uint8_t brefresh;      /*!< This field indicates the rate at which an isochronous synchronization pipe provides new
                              synchronization feedback data*/
    uint8_t bsynchaddress; /*!< Must be reset to zero*/
} usb_audio_stream_synch_endp_desc_t;

/*! @brief Audio class-specific as interface descriptor structure */
typedef struct _usb_audio_stream_spepific_as_intf_desc
{
    uint8_t blength;            /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of the descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of the descriptor*/
    uint8_t bterminallink; /*!< The Terminal ID of the Terminal to which the endpoint of this interface is connected*/
    uint8_t bdelay;        /*!< Expressed in number of frames*/
    uint8_t wformattag[2]; /*!< The Audio Data Format that has to be used to communicate with this interface*/
} usb_audio_stream_spepific_as_intf_desc_t;

/*! @brief Audio class-specific as interface descriptor structure */
typedef struct _usb_audio_2_0_stream_spepific_as_intf_desc
{
    uint8_t blength;            /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of the descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of the descriptor*/
    uint8_t bterminallink; /*!< The Terminal ID of the Terminal to which the endpoint of this interface is connected*/
    uint8_t bmControls;    /*!< D1..0: Active Alternate Setting Control
                                D3..2: Valid Alternate Settings Control
                                D7..4: Reserved. Must be set to 0.*/
    uint8_t bFormatType;   /*!< Constant identifying the Format Type the AudioStreaming interface is using.*/
    uint8_t bmFormats[4]; /*!< The Audio Data Format(s) that can be used to communicate with this interface. See the USB
                             Audio Data Formats document for further details*/
    uint8_t bNrChannels;  /*!< Number of physical channels in the AS Interface audio channel cluster.*/
    uint8_t bmChannelConfig[4]; /*!< Describes the spatial location of the physical channels.*/
    uint8_t biChannelNames;     /*!< Index of a string descriptor, describing the name of the first physical channel.*/
} usb_audio_2_0_stream_spepific_as_intf_desc_t;

/* Format type descriptor */
/*! @brief audio  Format type descriptor structure */
typedef struct _usb_audio_stream_format_type_desc
{
    uint8_t blength;            /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of the descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of the descriptor*/
    uint8_t bformattype;        /*!< Constant identifying the Format Type the AudioStreaming interface is using*/
    uint8_t bnrchannels;        /*!< Number of channels of device*/
    uint8_t bsubframesize;      /*!< Bytes per audio subframe*/
    uint8_t bbitresolution;     /*!< Bits per sample*/
    uint8_t bsamfreqtype;       /*!< Frequency supported*/
    uint8_t tsamfreq[1][3];     /*!< Sample frequency*/
} usb_audio_stream_format_type_desc_t;

/*! @brief audio  Format type descriptor structure */
typedef struct _usb_audio_2_0_stream_format_type_desc
{
    uint8_t blength;            /*!< Total size of the descriptor*/
    uint8_t bdescriptortype;    /*!< Descriptor type of the descriptor*/
    uint8_t bdescriptorsubtype; /*!< Subtype of the descriptor*/
    uint8_t bformattype;        /*!< Constant identifying the Format Type the AudioStreaming interface is using*/
    uint8_t bSubslotSize;       /*!< The number of bytes occupied by one audio subslot. Can be 1, 2, 3 or 4.*/
    uint8_t bBitResolution;     /*!< The number of effectively used bits from the available bits in an audio subslot*/
} usb_audio_2_0_stream_format_type_desc_t;

/*! @brief Audio instance structure and audio usb_host_class_handle pointer to this structure */
typedef struct _audio_instance
{
    usb_host_handle hostHandle;                  /*!< This instance's related host handle*/
    usb_device_handle deviceHandle;              /*!< This instance's related device handle*/
    usb_host_interface_handle streamIntfHandle;  /*!< This instance's audio stream interface handle*/
    usb_host_interface_handle controlIntfHandle; /*!< This instance's control stream interface handle*/
    void *asIntfDesc;                            /*!< Audio class class-specific as interface descriptor pointer*/
    void *formatTypeDesc;                        /*!< Audio class class-specific format type  descriptor pointer*/
    usb_descriptor_endpoint_t *isoEndpDesc; /*!< Audio class class-specific ISO audio data endpoint descriptor pointer*/
    usb_host_pipe_handle isoInPipe;         /*!< Audio class ISO in pipe*/
    usb_host_pipe_handle isoOutPipe;        /*!< Audio class ISO out pipe*/
    transfer_callback_t inCallbackFn;       /*!< Audio class ISO in transfer callback function*/
    void *inCallbackParam;                  /*!< Audio class ISO in transfer callback parameter*/
    transfer_callback_t outCallbackFn;      /*!< Audio class ISO out transfer callback function*/
    void *outCallbackParam;                 /*!< Audio class ISO out transfer callback function*/
    void *headerDesc;                       /*!< Audio class header descriptor pointer*/
    void *itDesc;                           /*!< Audio class input terminal descriptor pointer*/
    void *otDesc;                           /*!< Audio class output terminal descriptor pointer*/
    void *fuDesc;                           /*!< Audio class  feature unit descriptor pointer*/
    void *clockSource;                      /*!< Audio class clock source descriptor pointer*/
    usb_host_pipe_handle controlPipe;       /*!< Audio class  device control pipe*/
    transfer_callback_t controlCallbackFn;  /*!< Audio control transfer callback function*/
    void *controlCallbackParam;             /*!< Audio control transfer callback function*/
    usb_host_transfer_t *controlTransfer;   /*!< On-going control transfer*/
    uint16_t inPacketSize;                  /*!< Audio ISO in maximum packet size*/
    uint16_t outPacketSize;                 /*!< Audio ISO out maximum packet size*/
    uint16_t deviceAudioVersion;            /*!< device's current Audio version, 16bit to aligned with Spec*/
    uint8_t isSetup;                        /*!< Whether the audio setup transfer is transmitting*/
    uint8_t isoEpNum;                       /*!< Audio stream ISO endpoint number*/
    uint8_t streamIfnum;                    /*!< Audio stream ISO interface number*/

} audio_instance_t;

/*******************************************************************************
 * API
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @name USB host audio class APIs
 * @{
 */

/*!
 * @brief Initializes the audio instance.
 *
 * This function allocates the resource for the audio instance.
 *
 * @param deviceHandle       The device handle.
 * @param classHandlePtr Return class handle.
 *
 * @retval kStatus_USB_Success        The device is initialized successfully.
 * @retval kStatus_USB_AllocFail      Allocate memory fail.
 */
extern usb_status_t USB_HostAudioInit(usb_device_handle deviceHandle, usb_host_class_handle *classHandlePtr);

/*!
 * @brief Deinitializes the Audio instance.
 *
 * This function release the resource for audio instance.
 *
 * @param deviceHandle   The device handle.
 * @param classHandle The class handle.
 *
 * @retval kStatus_USB_Success        The device is deinitialized successfully.
 */
extern usb_status_t USB_HostAudioDeinit(usb_device_handle deviceHandle, usb_host_class_handle classHandle);

/*!
 * @brief Sets the audio class stream interface.
 *
 * This function binds the interface with the audio instance.
 *
 * @param classHandle        The class handle.
 * @param interfaceHandle    The interface handle.
 * @param alternateSetting   The alternate setting value.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam      The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success        The device is initialized successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy           There is no idle transfer.
 * @retval kStatus_USB_Error          Send transfer fail. See the USB_HostSendSetup.
 * @retval kStatus_USB_Busy           Callback return status, there is no idle pipe.
 * @retval kStatus_USB_TransferStall  Callback return status, the transfer is stalled by the device.
 * @retval kStatus_USB_Error          Callback return status, open pipe fail. See the USB_HostOpenPipe.
 */
extern usb_status_t USB_HostAudioStreamSetInterface(usb_host_class_handle classHandle,
                                                    usb_host_interface_handle interfaceHandle,
                                                    uint8_t alternateSetting,
                                                    transfer_callback_t callbackFn,
                                                    void *callbackParam);

/*!
 * @brief Sets the audio class control interface.
 *
 * This function binds the interface with the audio instance.
 *
 * @param classHandle        The class handle.
 * @param interfaceHandle    The interface handle.
 * @param alternateSetting   The alternate setting value.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam      The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success        The device is initialized successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy           There is no idle transfer.
 * @retval kStatus_USB_Error          Send transfer fail. See the USB_HostSendSetup.
 * @retval kStatus_USB_Busy           Callback return status, there is no idle pipe.
 * @retval kStatus_USB_TransferStall  Callback return status, the transfer is stalled by the device.
 * @retval kStatus_USB_Error          Callback return status, open pipe fail. See USB_HostOpenPipe.
 */
extern usb_status_t USB_HostAudioControlSetInterface(usb_host_class_handle classHandle,
                                                     usb_host_interface_handle interfaceHandle,
                                                     uint8_t alternateSetting,
                                                     transfer_callback_t callbackFn,
                                                     void *callbackParam);

/*!
 * @brief Gets the pipe maximum packet size.
 *
 * @param classHandle The class handle.
 * @param pipeType    Its value is USB_ENDPOINT_CONTROL, USB_ENDPOINT_ISOCHRONOUS, USB_ENDPOINT_BULK or
 * USB_ENDPOINT_INTERRUPT.
 *                     See the usb_spec.h
 * @param direction    Pipe direction.
 *
 * @retval 0        The classHandle is NULL.
 * @retval max      Packet size.
 */
extern uint16_t USB_HostAudioPacketSize(usb_host_class_handle classHandle, uint8_t pipeType, uint8_t direction);

/*!
 * @brief Audio stream receive data.
 *
 * This function implements the audio receiving data.
 *
 * @param classHandle      The class handle.
 * @param buffer            The buffer pointer.
 * @param bufferLen          The buffer length.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success          Receive request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error              Pipe is not initialized.
 *                                                       Or, send transfer fail. See the USB_HostRecv.
 */
extern usb_status_t USB_HostAudioStreamRecv(usb_host_class_handle classHandle,
                                            uint8_t *buffer,
                                            uint32_t bufferLen,
                                            transfer_callback_t callbackFn,
                                            void *callbackParam);

/*!
 * @brief Audio stream send data.
 *
 * This function implements the audio sending data.
 *
 * @param classHandle      The class handle.
 * @param buffer            The buffer pointer.
 * @param bufferLen          The buffer length.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success          Receive request successfully.
 * @retval kStatus_USB_InvalidHandle     The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error              pipe is not initialized.
 *                                                       Or, send transfer fail. See the USB_HostSend.
 */
extern usb_status_t USB_HostAudioStreamSend(usb_host_class_handle classHandle,
                                            uint8_t *buffer,
                                            uint32_t bufferLen,
                                            transfer_callback_t callbackFn,
                                            void *callbackParam);

/*!
 * @brief Gets the audio stream current altsetting descriptor.
 * @deprecated Do not use this function. It has been superceded by @ref
 * USB_HostAudioStreamGetCurrentAltsettingSpecificDescriptors.
 *
 * This function implements the get audio stream current altsetting descriptor.
 *
 * @param classHandle               The class handle.
 * @param asIntfDesc                The pointer of class-specific AS interface descriptor.
 * @param formatTypeDesc       		The pointer of format type descriptor.
 * @param isoEndpDesc               The pointer of specific ISO endp descriptor.
 *
 * @retval kStatus_USB_Success          Get the audio stream current altsetting descriptor request successfully.
 * @retval kStatus_USB_InvalidHandle   The classHandle is NULL pointer.
 *
 */
extern usb_status_t USB_HostAudioStreamGetCurrentAltsettingDescriptors(
    usb_host_class_handle classHandle,
    usb_audio_stream_spepific_as_intf_desc_t **asIntfDesc,
    usb_audio_stream_format_type_desc_t **formatTypeDesc,
    usb_descriptor_endpoint_t **isoEndpDesc);

/*!
 * @brief The USB audio feature unit request.
 * @deprecated Do not use this function. It has been superceded by @ref USB_HostAudioGetSetFeatureUnitRequest.
 *
 * This function implements the USB audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param channelNo        The channel number of audio feature unit.
 * @param buf              The feature unit request buffer pointer.
 * @param cmdCode          The feature unit command code, for example USB_AUDIO_GET_CUR_MUTE, and so on.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail. See the USB_HostSendSetup.
 *
 */
extern usb_status_t USB_HostAudioFeatureUnitRequest(usb_host_class_handle classHandle,
                                                    uint8_t channelNo,
                                                    void *buf,
                                                    uint32_t cmdCode,
                                                    transfer_callback_t callbackFn,
                                                    void *callbackParam);

/*!
 * @brief The USB audio endpoint request.
 * @deprecated Do not use this function. It has been superceded by @ref USB_HostAudioGetSetEndpointRequest.

 *
 * This function implements the USB audio endpoint request.
 *
 * @param classHandle      The class handle.
 * @param buf              The feature unit buffer pointer.
 * @param cmdCode          The feature unit command code, for example USB_AUDIO_GET_CUR_PITCH, and so on.
 * @param callbackFn         This callback is called after this function completes.
 * @param callbackParam   The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success          Endpoint request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error               Send transfer fail. See the USB_HostSendSetup.
 *
 */
extern usb_status_t USB_HostAudioEndpointRequest(usb_host_class_handle classHandle,
                                                 void *buf,
                                                 uint32_t cmdCode,
                                                 transfer_callback_t callbackFn,
                                                 void *callbackParam);
/*!
 * @brief get audio control current altsetting descriptor.
 *
 * This function implements get audio stream current altsetting descriptor.
 *
 * @param classHandle               The class handle.
 * @param DescriptorType            The descriptor type.
 * @param DescriptorSubType         The descriptor subtype, 0 for no subtype, for standard endpoint , 0 stand for data
 * endpoint.
 * @param Descriptor                The pointer of descriptor pointer.
 *
 * @retval kStatus_USB_Success          Get audio stream current altsetting descriptor request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 *
 */
usb_status_t USB_HostAudioControlGetCurrentAltsettingSpecificDescriptors(

    usb_host_class_handle classHandle, uint32_t DescriptorType, uint32_t DescriptorSubType, void **Descriptor);

/*!
 * @brief get audio control current altsetting descriptor.
 *
 * This function implements get audio stream current altsetting descriptor.
 *
 * @param classHandle               The class handle.
 * @param DescriptorType            The descriptor type.
 * @param DescriptorSubType         The descriptor subtype, 0 for no subtype.
 * @param Descriptor                The pointer of descriptor pointer.
 *
 * @retval kStatus_USB_Success          Get audio stream current altsetting descriptor request successfully.
 * @retval kStatus_USB_InvalidHandle  The classHandle is NULL pointer.
 *
 */
usb_status_t USB_HostAudioStreamGetCurrentAltsettingSpecificDescriptors(

    usb_host_class_handle classHandle, uint32_t DescriptorType, uint32_t DescriptorSubType, void **Descriptor);
/*!
 * @brief usb audio set/get feature unit request.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param csAndCn          The CS and CN or MCN for wValue field in setup Request.
 * @param cmdCode          The bRequest code in lower 8bit of lower word and get feature(1)/set feature(0) flag in
 * higher 8bit of lower word.
 * @param buf              The feature unit request buffer pointer.
 * @param callbackFn       This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioGetSetFeatureUnitRequest(usb_host_class_handle classHandle,
                                                   uint32_t csAndCn,
                                                   uint32_t cmdCode,
                                                   void *buf,
                                                   uint32_t bufLen,
                                                   transfer_callback_t callbackFn,
                                                   void *callbackParam);

/*!
 * @brief usb audio set/get feature unit request.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param csAndCn          The CS and CN or MCN for wValue field in setup Request.
 * @param cmdCode          The bRequest code in lower 8bit of lower word and get clock(1)/set clock(0) flag in higher
 * 8bit of lower word.
 * @param buf              The feature unit request buffer pointer.
 * @param callbackFn       This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioGetSetClockSourceRequest(usb_host_class_handle classHandle,
                                                   uint32_t csAndCn,
                                                   uint32_t cmdCode,
                                                   void *buf,
                                                   uint32_t bufLen,
                                                   transfer_callback_t callbackFn,
                                                   void *callbackParam);
/*!
 * @brief usb audio set/get endp unit request.
 *
 * This function implements usb audio feature unit request.
 *
 * @param classHandle      The class handle.
 * @param csAndCn          The CS for wValue field in setup Request.
 * @param cmdCode          The bRequest code in lower 8bit of lower word and get(1)/set(0) flag in higher 8bit of lower
 word.
 * @param buf              The feature unit request buffer pointer.

 * @param callbackFn       This callback is called after this function completes.
 * @param callbackParam  The first parameter in the callback function.
 *
 * @retval kStatus_USB_Success            Feature unit request successfully.
 * @retval kStatus_USB_InvalidHandle    The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy                There is no idle transfer.
 * @retval kStatus_USB_Error                Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioGetSetEndpointRequest(usb_host_class_handle classHandle,
                                                uint32_t csAndCn,
                                                uint32_t cmdCode,
                                                void *buf,
                                                uint32_t bufLen,
                                                transfer_callback_t callbackFn,
                                                void *callbackParam);

/*!
 * @brief change the ISO out data interval
 *
 * when the low interval can satisfy the device's data bandwidth requirement, the interval can be increased to decrease
 * the MCU loading. for example: the audio speaker device is 48K/2channels/2Bytes and the original interval is 125us,
 * mps is 256Bytes. If using the 125us interval, the USB interrupt will trigger every 125us, it need much MCU loading
 *              (in FreeRTOS environment especially because there is task switch time).
 *              Change the interval as 1ms, it sill can satisfy the device's data bandwidth requirement as follow:
 *              the data lenght is 48 * 2 * 2 = 192Bytes every ms, and the 256Bytes (mps) is bigger than 192Bytes, so
 * the inteval can be changed to 1ms. Then host sends 192Bytes in one micro-frame of the 8 micro-frames (1ms), and there
 * are no data transfers in the other 7 micro-frames.
 *
 * @param classHandle      The class handle.
 * @param intervalValue    The new interval value according to the interval descriptor.
 *
 * @retval kStatus_USB_Success            the change request successfully.
 * @retval kStatus_USB_InvalidHandle      The classHandle is NULL pointer.
 * @retval kStatus_USB_Busy               There is no idle transfer.
 * @retval kStatus_USB_Error              Send transfer fail, please reference to USB_HostSendSetup.
 *
 */
usb_status_t USB_HostAudioSetStreamOutDataInterval(usb_host_class_handle classHandle, uint8_t intervalValue);

/*! @}*/
#ifdef __cplusplus
}
#endif
/*! @}*/
#endif /* __USB_HOST_AUDIO_H__ */
