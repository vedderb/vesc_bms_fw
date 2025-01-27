/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __USB_DEVICE_AUDIO_H__
#define __USB_DEVICE_AUDIO_H__

/*!
 * @addtogroup usb_device_audio_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*!
 * @name USB Audio class codes
 * @{
 */

/*! @brief Audio device class code */
#define USB_DEVICE_CONFIG_AUDIO_CLASS_CODE (0x01U)

/*! @brief Audio device subclass code */
#define USB_DEVICE_AUDIO_STREAM_SUBCLASS  (0x02U)
#define USB_DEVICE_AUDIO_CONTROL_SUBCLASS (0x01U)

/*! @brief Audio device class-specific descriptor type */
#define USB_DESCRIPTOR_TYPE_AUDIO_CS_INTERFACE (0x24)
#define USB_DESCRIPTOR_TYPE_AUDIO_CS_ENDPOINT  (0x25)

/*! @brief Audio device class-specific control interface descriptor subtype */
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_HEADER          (0x01)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_INPUT_TERMINAL  (0x02)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_OUTPUT_TERMINAL (0x03)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_MIXER_UNIT      (0x04)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_SELECTOR_UNIT   (0x05)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_FEATURE_UNIT    (0x06)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_EFFECT_UNIT                (0x07)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_PROCESSING_UNIT_AUDIO20    (0x08)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_EXTENSION_UNIT_AUDIO20     (0x09)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SOURCE_UNIT          (0x0A)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_SELECTOR_UNIT        (0x0B)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_CLOCK_MULTIPLIER_UNIT      (0x0C)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_SAMPLE_RATE_CONVERTER_UNIT (0x0D)
#else
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_PROCESSING_UNIT (0x07)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_CONTROL_EXTENSION_UNIT  (0x08)
#endif

/*! @brief Audio device class-specific control interface effect unit effect type */
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DESCRIPTOR_AUDIO_CONTROL_EFFECT_UNIT_UNDEFINED_TYPE               (0x00)
#define USB_DESCRIPTOR_AUDIO_CONTROL_EFFECT_UNIT_PARAM_EQ_SECTION_EFFECT_TYPE (0x01)
#define USB_DESCRIPTOR_AUDIO_CONTROL_EFFECT_UNIT_REVERBERATION_EFFECT_TYPE    (0x02)
#define USB_DESCRIPTOR_AUDIO_CONTROL_EFFECT_UNIT_MOD_DELAY_EFFECT_TYPE        (0x03)
#define USB_DESCRIPTOR_AUDIO_CONTROL_EFFECT_UNIT_DYN_RANGE_COMP_EFFECT_TYPE   (0x04)
#endif

/*! @brief Audio device class-specific control interface processing unit process type */
#define USB_DESCRIPTOR_AUDIO_CONTROL_PROCESSING_UNIT_UPDOWNMIX_PROCESS_TYPE       (0x01)
#define USB_DESCRIPTOR_AUDIO_CONTROL_PROCESSING_UNIT_DOLBY_PROLOGIC_PROCESS_TYPE  (0x02)
#define USB_DESCRIPTOR_AUDIO_CONTROL_PROCESSING_UNIT_STEREO_EXTENDER_PROCESS_TYPE (0x03)
#if (!USB_DEVICE_CONFIG_AUDIO_CLASS_2_0) /* used by usb audio 1.0 */
#define USB_DESCRIPTOR_AUDIO_CONTROL_PROCESSING_UNIT_REVERBERATION_PROCESS_TYPE  (0x04)
#define USB_DESCRIPTOR_AUDIO_CONTROL_PROCESSING_UNIT_CHORUS_PROCESS_TYPE         (0x05)
#define USB_DESCRIPTOR_AUDIO_CONTROL_PROCESSING_UNIT_DYN_RANGE_COMP_PROCESS_TYPE (0x06)
#endif

/*! @brief Audio device class-specific stream interface descriptor subtype */
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_AS_GENERAL  (0x01)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_FORMAT_TYPE (0x02)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_ENCODER (0x03)
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_DECODER (0x04)
#else
#define USB_DESCRIPTOR_SUBTYPE_AUDIO_STREAMING_FORMAT_SPECIFIC (0x03)
#endif

/*! @brief Audio device Format Type Codes */
#define USB_AUDIO_FORMAT_TYPE_UNDEFINED (0x00)
#define USB_AUDIO_FORMAT_TYPE_I         (0x01)
#define USB_AUDIO_FORMAT_TYPE_II        (0x02)
#define USB_AUDIO_FORMAT_TYPE_III       (0x03)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_AUDIO_FORMAT_TYPE_IV             (0x04)
#define USB_AUDIO_FORMAT_EXT_FORMAT_TYPE_I   (0x81)
#define USB_AUDIO_FORMAT_EXT_FORMAT_TYPE_II  (0x82)
#define USB_AUDIO_FORMAT_EXT_FORMAT_TYPE_III (0x83)
#endif

/*! @brief Audio device class-specific stream interface Encoder/Decoder Type Codes */
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DESCRIPTOR_AUDIO_STREAM_ENCODER_UNDEFINED     (0x00)
#define USB_DESCRIPTOR_AUDIO_STREAM_ENCODER_OTHER_ENCODER (0x01)
#define USB_DESCRIPTOR_AUDIO_STREAM_ENCODER_MPEG_ENCODER  (0x02)
#define USB_DESCRIPTOR_AUDIO_STREAM_ENCODER_AC3_ENCODER   (0x03)
#define USB_DESCRIPTOR_AUDIO_STREAM_ENCODER_WMA_ENCODER   (0x04)
#define USB_DESCRIPTOR_AUDIO_STREAM_ENCODER_DTS_ENCODER   (0x05)

#define USB_DESCRIPTOR_AUDIO_STREAM_DECODER_UNDEFINED     (0x00)
#define USB_DESCRIPTOR_AUDIO_STREAM_DECODER_OTHER_DECODER (0x01)
#define USB_DESCRIPTOR_AUDIO_STREAM_DECODER_MPEG_DECODER  (0x02)
#define USB_DESCRIPTOR_AUDIO_STREAM_DECODER_AC3_DECODER   (0x03)
#define USB_DESCRIPTOR_AUDIO_STREAM_DECODER_WMA_DECODER   (0x04)
#define USB_DESCRIPTOR_AUDIO_STREAM_DECODER_DTS_DECODER   (0x05)
#endif

/*! @brief Commands for USB device AUDIO Class specific request codes */
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_CUR_REQUEST   (0x01U)
#define USB_DEVICE_AUDIO_RANGE_REQUEST (0x02U)
#define USB_DEVICE_AUDIO_MEM_REQUEST   (0x03U)
#else
#define USB_DEVICE_AUDIO_SET_CUR_REQUEST (0x01)
#define USB_DEVICE_AUDIO_GET_CUR_REQUEST (0x81)
#define USB_DEVICE_AUDIO_SET_MIN_REQUEST (0x02)
#define USB_DEVICE_AUDIO_GET_MIN_REQUEST (0x82)
#define USB_DEVICE_AUDIO_SET_MAX_REQUEST (0x03)
#define USB_DEVICE_AUDIO_GET_MAX_REQUEST (0x83)
#define USB_DEVICE_AUDIO_SET_RES_REQUEST (0x04)
#define USB_DEVICE_AUDIO_GET_RES_REQUEST (0x84)
#define USB_DEVICE_AUDIO_SET_MEM_REQUEST (0x05)
#define USB_DEVICE_AUDIO_GET_MEM_REQUEST (0x85)
#endif

/*! @brief Commands for USB device AUDIO control feature unit control selector */
#define USB_DEVICE_AUDIO_FU_MUTE_CONTROL_SELECTOR              (0x01)
#define USB_DEVICE_AUDIO_FU_VOLUME_CONTROL_SELECTOR            (0x02)
#define USB_DEVICE_AUDIO_FU_BASS_CONTROL_SELECTOR              (0x03)
#define USB_DEVICE_AUDIO_FU_MID_CONTROL_SELECTOR               (0x04)
#define USB_DEVICE_AUDIO_FU_TREBLE_CONTROL_SELECTOR            (0x05)
#define USB_DEVICE_AUDIO_FU_GRAPHIC_EQUALIZER_CONTROL_SELECTOR (0x06)
#define USB_DEVICE_AUDIO_FU_AUTOMATIC_GAIN_CONTROL_SELECTOR    (0x07)
#define USB_DEVICE_AUDIO_FU_DELAY_CONTROL_SELECTOR             (0x08)
#define USB_DEVICE_AUDIO_FU_BASS_BOOST_CONTROL_SELECTOR        (0x09)
#define USB_DEVICE_AUDIO_FU_LOUDNESS_CONTROL_SELECTOR          (0x0A)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_FU_INPUT_GAIN_CONTROL_SELECTOR     (0x0B)
#define USB_DEVICE_AUDIO_FU_INPUT_GAIN_PAD_CONTROL_SELECTOR (0x0C)
#define USB_DEVICE_AUDIO_FU_PHASE_INVERTER_CONTROL_SELECTOR (0x0D)
#define USB_DEVICE_AUDIO_FU_UNDERFLOW_CONTROL_SELECTOR      (0x0E)
#define USB_DEVICE_AUDIO_FU_OVERFLOW_CONTROL_SELECTOR       (0x0F)
#define USB_DEVICE_AUDIO_FU_LATENCY_CONTROL_SELECTOR        (0x10)
#endif

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
/*! @brief Commands for USB device AUDIO control Parametric Equalizer Section Effect Unit Control Selectors  */
#define USB_DEVICE_AUDIO_EU_PE_ENABLE_CONTROL     (0x01)
#define USB_DEVICE_AUDIO_EU_PE_CENTERFREQ_CONTROL (0x02)
#define USB_DEVICE_AUDIO_EU_PE_QFACTOR_CONTROL    (0x03)
#define USB_DEVICE_AUDIO_EU_PE_GAIN_CONTROL       (0x04)
#define USB_DEVICE_AUDIO_EU_PE_UNDERFLOW_CONTROL  (0x05)
#define USB_DEVICE_AUDIO_EU_PE_OVERFLOW_CONTROL   (0x06)
#define USB_DEVICE_AUDIO_EU_PE_LATENCY_CONTROL    (0x07)

/*! @brief Commands for USB device AUDIO control Reverberation Effect Unit Control Selectors Control Selectors  */
#define USB_DEVICE_AUDIO_EU_RV_ENABLE_CONTROL            (0x01)
#define USB_DEVICE_AUDIO_EU_RV_TYPE_CONTROL              (0x02)
#define USB_DEVICE_AUDIO_EU_RV_LEVEL_CONTROL             (0x03)
#define USB_DEVICE_AUDIO_EU_RV_TIME_CONTROL              (0x04)
#define USB_DEVICE_AUDIO_EU_RV_FEEDBACK_CONTROL          (0x05)
#define USB_DEVICE_AUDIO_EU_RV_PREDELAY_CONTROL          (0x06)
#define USB_DEVICE_AUDIO_EU_RV_DENSITY_CONTROL           (0x07)
#define USB_DEVICE_AUDIO_EU_RV_RV_HIFREQ_ROLLOFF_CONTROL (0x08)
#define USB_DEVICE_AUDIO_EU_RV_UNDERFLOW_CONTROL         (0x09)
#define USB_DEVICE_AUDIO_EU_RV_OVERFLOW_CONTROL          (0x0A)
#define USB_DEVICE_AUDIO_EU_RV_LATENCY_CONTROL           (0x0B)

/*! @brief Commands for USB device AUDIO control Modulation Delay Effect Unit Control Selectors  */
#define USB_DEVICE_AUDIO_EU_MD_ENABLE_CONTROL    (0x01)
#define USB_DEVICE_AUDIO_EU_MD_BALANCE_CONTROL   (0x02)
#define USB_DEVICE_AUDIO_EU_MD_RATE_CONTROL      (0x03)
#define USB_DEVICE_AUDIO_EU_MD_DEPTH_CONTROL     (0x04)
#define USB_DEVICE_AUDIO_EU_MD_TIME_CONTROL      (0x05)
#define USB_DEVICE_AUDIO_EU_MD_FEEDBACK_CONTROL  (0x06)
#define USB_DEVICE_AUDIO_EU_MD_UNDERFLOW_CONTROL (0x07)
#define USB_DEVICE_AUDIO_EU_MD_OVERFLOW_CONTROL  (0x08)
#define USB_DEVICE_AUDIO_EU_MD_LATENCY_CONTROL   (0x09)

/*! @brief Commands for USB device AUDIO control Dynamic Range Compressor Effect Unit Control Selectors */
#define USB_DEVICE_AUDIO_EU_DR_ENABLE_CONTROL           (0x01)
#define USB_DEVICE_AUDIO_EU_DR_COMPRESSION_RATE_CONTROL (0x02)
#define USB_DEVICE_AUDIO_EU_DR_MAXAMPL_CONTROL          (0x03)
#define USB_DEVICE_AUDIO_EU_DR_THRESHOLD_CONTROL        (0x04)
#define USB_DEVICE_AUDIO_EU_DR_ATTACK_TIME_CONTROL      (0x05)
#define USB_DEVICE_AUDIO_EU_DR_RELEASE_TIME_CONTROL     (0x06)
#define USB_DEVICE_AUDIO_EU_DR_UNDERFLOW_CONTROL        (0x07)
#define USB_DEVICE_AUDIO_EU_DR_OVERFLOW_CONTROL         (0x08)
#define USB_DEVICE_AUDIO_EU_DR_LATENCY_CONTROL          (0x09)
#endif

/*! @brief Commands for USB device AUDIO control Up/Down-mix Processing Unit Control Selectors */
#define USB_DEVICE_AUDIO_PU_UD_ENABLE_CONTROL      (0x01)
#define USB_DEVICE_AUDIO_PU_UD_MODE_SELECT_CONTROL (0x02)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_PU_UD_CLUSTER_CONTROL   (0x03)
#define USB_DEVICE_AUDIO_PU_UD_UNDERFLOW_CONTROL (0x04)
#define USB_DEVICE_AUDIO_PU_UD_OVERFLOW_CONTROL  (0x05)
#define USB_DEVICE_AUDIO_PU_UD_LATENCY_CONTROL   (0x06)
#endif

/*! @brief Commands for USB device AUDIO control Dolby Prologic Processing Unit Control Selectors */
#define USB_DEVICE_AUDIO_PU_DP_ENABLE_CONTROL      (0x01)
#define USB_DEVICE_AUDIO_PU_DP_MODE_SELECT_CONTROL (0x02)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_PU_DP_CLUSTER_CONTROL   (0x03)
#define USB_DEVICE_AUDIO_PU_DP_UNDERFLOW_CONTROL (0x04)
#define USB_DEVICE_AUDIO_PU_DP_OVERFLOW_CONTROL  (0x05)
#define USB_DEVICE_AUDIO_PU_DP_LATENCY_CONTROL   (0x06)
#endif

/*! @brief Commands for USB device AUDIO control (3D,audio 1.0) Stereo Extender Processing Unit Control Selectors */
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_PU_ST_EXT_ENABLE_CONTROL    (0x01)
#define USB_DEVICE_AUDIO_PU_ST_EXT_WIDTH_CONTROL     (0x02)
#define USB_DEVICE_AUDIO_PU_ST_EXT_UNDERFLOW_CONTROL (0x03)
#define USB_DEVICE_AUDIO_PU_ST_EXT_OVERFLOW_CONTROL  (0x04)
#define USB_DEVICE_AUDIO_PU_ST_EXT_LATENCY_CONTROL   (0x05)
#else
#define USB_DEVICE_AUDIO_PU_3D_ENABLE_CONTROL    (0x01)
#define USB_DEVICE_AUDIO_PU_SPACIOUSNESS_CONTROL (0x03)
#endif

#if (!USB_DEVICE_CONFIG_AUDIO_CLASS_2_0) /* USB audio 1.0 */
/*! @brief Commands for USB device AUDIO control Reverberation Processing Unit Control Selectors */
#define USB_DEVICE_AUDIO_PU_RV_ENABLE_CONTROL   (0x01)
#define USB_DEVICE_AUDIO_PU_RV_LEVEL_CONTROL    (0x02)
#define USB_DEVICE_AUDIO_PU_RV_TIME_CONTROL     (0x03)
#define USB_DEVICE_AUDIO_PU_RV_FEEDBACK_CONTROL (0x04)

/*! @brief Commands for USB device AUDIO control Chorus Processing Unit Control Selectors */
#define USB_DEVICE_AUDIO_PU_CH_ENABLE_CONTROL (0x01)
#define USB_DEVICE_AUDIO_PU_CH_LEVEL_CONTROL  (0x02)
#define USB_DEVICE_AUDIO_PU_CH_RATE_CONTROL   (0x03)
#define USB_DEVICE_AUDIO_PU_CH_DEPTH_CONTROL  (0x04)

/*! @brief Commands for USB device AUDIO control Dynamic Range Compressor Processing Unit Control Selectors */
#define USB_DEVICE_AUDIO_PU_DR_ENABLE_CONTROL           (0x01)
#define USB_DEVICE_AUDIO_PU_DR_COMPRESSION_RATE_CONTROL (0x02)
#define USB_DEVICE_AUDIO_PU_DR_MAXAMPL_CONTROL          (0x03)
#define USB_DEVICE_AUDIO_PU_DR_THRESHOLD_CONTROL        (0x04)
#define USB_DEVICE_AUDIO_PU_DR_ATTACK_TIME              (0x05)
#define USB_DEVICE_AUDIO_PU_DR_RELEASE_TIME             (0x06)
#endif

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
/*! @brief Commands for USB device AUDIO control clock source control selector */
#define USB_DEVICE_AUDIO_CS_CONTROL_UNDEFINED            (0x00U)
#define USB_DEVICE_AUDIO_CS_SAM_FREQ_CONTROL_SELECTOR    (0x01U)
#define USB_DEVICE_AUDIO_CS_CLOCK_VALID_CONTROL_SELECTOR (0x02U)

/*! @brief Commands for USB device AUDIO control clock selector control selector */
#define USB_DEVICE_AUDIO_CX_CONTROL_UNDEFINED               (0x00U)
#define USB_DEVICE_AUDIO_CX_CLOCK_SELECTOR_CONTROL_SELECTOR (0x01U)

/*! @brief Commands for USB device AUDIO control clock multiplier control selector */
#define USB_DEVICE_AUDIO_CM_CONTROL_UNDEFINED            (0x00U)
#define USB_DEVICE_AUDIO_CM_NUMERATOR_CONTROL_SELECTOR   (0x01U)
#define USB_DEVICE_AUDIO_CM_DENOMINATOR_CONTROL_SELECTOR (0x02U)
#endif

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
/*! @brief Commands for USB device AUDIO streaming Interface Control Selectors */
#define USB_DEVICE_AUDIO_AS_ACT_ALT_SETTING_CONTROL   (0x01U)
#define USB_DEVICE_AUDIO_AS_VAL_ALT_SETTINGS_CONTROL  (0x02U)
#define USB_DEVICE_AUDIO_AS_AUDIO_DATA_FORMAT_CONTROL (0x03U)

/*! @brief Commands for USB device AUDIO streaming Encoder control selector */
#define USB_DEVICE_AUDIO_EN_BIT_RATE_CONTROL      (0x01U)
#define USB_DEVICE_AUDIO_EN_QUALITY_CONTROL       (0x02U)
#define USB_DEVICE_AUDIO_EN_VBR_CONTROL           (0x03U)
#define USB_DEVICE_AUDIO_EN_TYPE_CONTROL          (0x04U)
#define USB_DEVICE_AUDIO_EN_UNDERFLOW_CONTROL     (0x05U)
#define USB_DEVICE_AUDIO_EN_OVERFLOW_CONTROL      (0x06U)
#define USB_DEVICE_AUDIO_EN_ENCODER_ERROR_CONTROL (0x07U)
#define USB_DEVICE_AUDIO_EN_PARAM1_CONTROL        (0x08U)
#define USB_DEVICE_AUDIO_EN_PARAM2_CONTROL        (0x09U)
#define USB_DEVICE_AUDIO_EN_PARAM3_CONTROL        (0x0AU)
#define USB_DEVICE_AUDIO_EN_PARAM4_CONTROL        (0x0BU)
#define USB_DEVICE_AUDIO_EN_PARAM5_CONTROL        (0x0CU)
#define USB_DEVICE_AUDIO_EN_PARAM6_CONTROL        (0x0DU)
#define USB_DEVICE_AUDIO_EN_PARAM7_CONTROL        (0x0EU)
#define USB_DEVICE_AUDIO_EN_PARAM8_CONTROL        (0x0FU)

/*! @brief Commands for USB device AUDIO streaming MPEG Decoder control selector */
#define USB_DEVICE_AUDIO_MD_DUAL_CHANNEL_CONTROL  (0x01U)
#define USB_DEVICE_AUDIO_MD_SECOND_STEREO_CONTROL (0x02U)
#define USB_DEVICE_AUDIO_MD_MULTILINGUAL_CONTROL  (0x03U)
#define USB_DEVICE_AUDIO_MD_DYN_RANGE_CONTROL     (0x04U)
#define USB_DEVICE_AUDIO_MD_SCALING_CONTROL       (0x05U)
#define USB_DEVICE_AUDIO_MD_HILO_SCALING_CONTROL  (0x06U)
#define USB_DEVICE_AUDIO_MD_UNDERFLOW_CONTROL     (0x07U)
#define USB_DEVICE_AUDIO_MD_OVERFLOW_CONTROL      (0x08U)
#define USB_DEVICE_AUDIO_MD_DECODER_ERROR_CONTROL (0x09U)

/*! @brief Commands for USB device AUDIO streaming AC-3 Decoder Control Selectors */
#define USB_DEVICE_AUDIO_AD_MODE_CONTROL          (0x01U)
#define USB_DEVICE_AUDIO_AD_DYN_RANGE_CONTROL     (0x02U)
#define USB_DEVICE_AUDIO_AD_SCALING_CONTROL       (0x03U)
#define USB_DEVICE_AUDIO_AD_HILO_SCALING_CONTROLL (0x04U)
#define USB_DEVICE_AUDIO_AD_UNDERFLOW_CONTROL     (0x05U)
#define USB_DEVICE_AUDIO_AD_OVERFLOW_CONTROL      (0x06U)
#define USB_DEVICE_AUDIO_AD_DECODER_ERROR_CONTROL (0x07U)

/*! @brief Commands for USB device AUDIO streaming WMA Decoder Control Selectors */
#define USB_DEVICE_AUDIO_WD_UNDERFLOW_CONTROL     (0x01U)
#define USB_DEVICE_AUDIO_WD_OVERFLOW_CONTROL      (0x02U)
#define USB_DEVICE_AUDIO_WD_DECODER_ERROR_CONTROL (0x03U)

/*! @brief Commands for USB device AUDIO streaming DTS Decoder Control Selectors */
#define USB_DEVICE_AUDIO_DD_UNDERFLOW_CONTROL     (0x01U)
#define USB_DEVICE_AUDIO_DD_OVERFLOW_CONTROL      (0x02U)
#define USB_DEVICE_AUDIO_DD_DECODER_ERROR_CONTROL (0x03U)
#endif

#if (!USB_DEVICE_CONFIG_AUDIO_CLASS_2_0) /* USB audio 1.0 */
/*! @brief Commands for USB device AUDIO streaming MPEG control selector */
#define USB_DEVICE_AUDIO_MP_DUAL_CHANNEL_CONTROL  (0x01)
#define USB_DEVICE_AUDIO_MP_SECOND_STEREO_CONTROL (0x02)
#define USB_DEVICE_AUDIO_MP_MULTILINGUAL_CONTROL  (0x03)
#define USB_DEVICE_AUDIO_MP_DYN_RANGE_CONTROL     (0x04)
#define USB_DEVICE_AUDIO_MP_SCALING_CONTROL       (0x05)
#define USB_DEVICE_AUDIO_MP_HILO_SCALING_CONTROL  (0x06)

/*! @brief Commands for USB device AUDIO streaming AC-3 Control Selectors */
#define USB_DEVICE_AUDIO_AC_MODE_CONTROL     (0x01)
#define USB_DEVICE_AUDIO_AC_DYN_RANGE_CONTRO (0x02)
#define USB_DEVICE_AUDIO_AC_SCALING_CONTROL  (0x03)
#define USB_DEVICE_AUDIO_AC_HILO_SCALING_CONTROL (0x04)
#endif

/*! @brief Commands for USB device AUDIO streaming endpoint control selector */
#define USB_DEVICE_AUDIO_EP_CONTROL_UNDEFINED (0x00)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_EP_PITCH_CONTROL_SELECTOR_AUDIO20 (0x01)
#define USB_DEVICE_AUDIO_EP_DATA_OVERRUN_CONTROL_SELECTOR  (0x02)
#define USB_DEVICE_AUDIO_EP_DATA_UNDERRUN_CONTROL_SELECTOR (0x03)
#else
#define USB_DEVICE_AUDIO_EP_SAMPLING_FREQ_CONTROL_SELECTOR (0x01)
#define USB_DEVICE_AUDIO_EP_PITCH_CONTROL_SELECTOR         (0x02)
#endif

/* Commands for USB device AUDIO terminal control selector */
#define USB_DEVICE_AUDIO_TE_CONTROL_UNDEFINED (0x00)
#define USB_DEVICE_AUDIO_TE_COPY_PROTECT_CONTROL (0x01)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_TE_CONNECTOR_CONTROL (0x02)
#define USB_DEVICE_AUDIO_TE_OVERLOAD_CONTROL (0x03)
#define USB_DEVICE_AUDIO_TE_CLUSTER_CONTROL (0x04)
#define USB_DEVICE_AUDIO_TE_UNDERFLOW_CONTROL (0x05)
#define USB_DEVICE_AUDIO_TE_OVERFLOW_CONTROL (0x06)
#define USB_DEVICE_AUDIO_TE_LATENCY_CONTROL (0x07)
#endif
   
/*! @brief Audio device class-specific FU GET CUR COMMAND  */
#define USB_DEVICE_AUDIO_FU_GET_CUR_MUTE_CONTROL              (0x8101)
#define USB_DEVICE_AUDIO_FU_GET_CUR_VOLUME_CONTROL            (0x8102)
#define USB_DEVICE_AUDIO_FU_GET_CUR_BASS_CONTROL              (0x8103)
#define USB_DEVICE_AUDIO_FU_GET_CUR_MID_CONTROL               (0x8104)
#define USB_DEVICE_AUDIO_FU_GET_CUR_TREBLE_CONTROL            (0x8105)
#define USB_DEVICE_AUDIO_FU_GET_CUR_GRAPHIC_EQUALIZER_CONTROL (0x8106)
#define USB_DEVICE_AUDIO_FU_GET_CUR_AUTOMATIC_GAIN_CONTROL    (0x8107)
#define USB_DEVICE_AUDIO_FU_GET_CUR_DELAY_CONTROL             (0x8108)
#define USB_DEVICE_AUDIO_FU_GET_CUR_BASS_BOOST_CONTROL        (0x8109)
#define USB_DEVICE_AUDIO_FU_GET_CUR_LOUDNESS_CONTROL          (0x810A)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_FU_GET_CUR_UNDERFLOW_CONTROL (0x810E)
#define USB_DEVICE_AUDIO_FU_GET_CUR_OVERFLOW_CONTROL  (0x810F)
#endif

/*! @brief Audio device class-specific FU GET MIN COMMAND  */
#define USB_DEVICE_AUDIO_FU_GET_MIN_VOLUME_CONTROL            (0x8202)
#define USB_DEVICE_AUDIO_FU_GET_MIN_BASS_CONTROL              (0x8203)
#define USB_DEVICE_AUDIO_FU_GET_MIN_MID_CONTROL               (0x8204)
#define USB_DEVICE_AUDIO_FU_GET_MIN_TREBLE_CONTROL            (0x8205)
#define USB_DEVICE_AUDIO_FU_GET_MIN_GRAPHIC_EQUALIZER_CONTROL (0x8206)
#define USB_DEVICE_AUDIO_FU_GET_MIN_DELAY_CONTROL             (0x8208)

/*! @brief Audio device class-specific FU GET MAX COMMAND  */
#define USB_DEVICE_AUDIO_FU_GET_MAX_VOLUME_CONTROL            (0x8302)
#define USB_DEVICE_AUDIO_FU_GET_MAX_BASS_CONTROL              (0x8303)
#define USB_DEVICE_AUDIO_FU_GET_MAX_MID_CONTROL               (0x8304)
#define USB_DEVICE_AUDIO_FU_GET_MAX_TREBLE_CONTROL            (0x8305)
#define USB_DEVICE_AUDIO_FU_GET_MAX_GRAPHIC_EQUALIZER_CONTROL (0x8306)
#define USB_DEVICE_AUDIO_FU_GET_MAX_DELAY_CONTROL             (0x8308)

/*! @brief Audio device class-specific FU GET RES COMMAND  */
#define USB_DEVICE_AUDIO_FU_GET_RES_VOLUME_CONTROL            (0x8402)
#define USB_DEVICE_AUDIO_FU_GET_RES_BASS_CONTROL              (0x8403)
#define USB_DEVICE_AUDIO_FU_GET_RES_MID_CONTROL               (0x8404)
#define USB_DEVICE_AUDIO_FU_GET_RES_TREBLE_CONTROL            (0x8405)
#define USB_DEVICE_AUDIO_FU_GET_RES_GRAPHIC_EQUALIZER_CONTROL (0x8406)
#define USB_DEVICE_AUDIO_FU_GET_RES_DELAY_CONTROL             (0x8408)

/*! @brief Audio device class-specific FU SET CUR COMMAND  */
#define USB_DEVICE_AUDIO_FU_SET_CUR_MUTE_CONTROL              (0x0101)
#define USB_DEVICE_AUDIO_FU_SET_CUR_VOLUME_CONTROL            (0x0102)
#define USB_DEVICE_AUDIO_FU_SET_CUR_BASS_CONTROL              (0x0103)
#define USB_DEVICE_AUDIO_FU_SET_CUR_MID_CONTROL               (0x0104)
#define USB_DEVICE_AUDIO_FU_SET_CUR_TREBLE_CONTROL            (0x0105)
#define USB_DEVICE_AUDIO_FU_SET_CUR_GRAPHIC_EQUALIZER_CONTROL (0x0106)
#define USB_DEVICE_AUDIO_FU_SET_CUR_AUTOMATIC_GAIN_CONTROL    (0x0107)
#define USB_DEVICE_AUDIO_FU_SET_CUR_DELAY_CONTROL             (0x0108)
#define USB_DEVICE_AUDIO_FU_SET_CUR_BASS_BOOST_CONTROL        (0x0109)
#define USB_DEVICE_AUDIO_FU_SET_CUR_LOUDNESS_CONTROL          (0x010A)

/*! @brief Audio device class-specific FU SET MIN COMMAND  */
#define USB_DEVICE_AUDIO_FU_SET_MIN_VOLUME_CONTROL            (0x0202)
#define USB_DEVICE_AUDIO_FU_SET_MIN_BASS_CONTROL              (0x0203)
#define USB_DEVICE_AUDIO_FU_SET_MIN_MID_CONTROL               (0x0204)
#define USB_DEVICE_AUDIO_FU_SET_MIN_TREBLE_CONTROL            (0x0205)
#define USB_DEVICE_AUDIO_FU_SET_MIN_GRAPHIC_EQUALIZER_CONTROL (0x0206)
#define USB_DEVICE_AUDIO_FU_SET_MIN_DELAY_CONTROL             (0x0208)

/*! @brief Audio device class-specific FU SET MAX COMMAND  */
#define USB_DEVICE_AUDIO_FU_SET_MAX_VOLUME_CONTROL            (0x0302)
#define USB_DEVICE_AUDIO_FU_SET_MAX_BASS_CONTROL              (0x0303)
#define USB_DEVICE_AUDIO_FU_SET_MAX_MID_CONTROL               (0x0304)
#define USB_DEVICE_AUDIO_FU_SET_MAX_TREBLE_CONTROL            (0x0305)
#define USB_DEVICE_AUDIO_FU_SET_MAX_GRAPHIC_EQUALIZER_CONTROL (0x0306)
#define USB_DEVICE_AUDIO_FU_SET_MAX_DELAY_CONTROL             (0x0308)

/*! @brief Audio device class-specific FU SET RES COMMAND  */
#define USB_DEVICE_AUDIO_FU_SET_RES_VOLUME_CONTROL            (0x0402)
#define USB_DEVICE_AUDIO_FU_SET_RES_BASS_CONTROL              (0x0403)
#define USB_DEVICE_AUDIO_FU_SET_RES_MID_CONTROL               (0x0404)
#define USB_DEVICE_AUDIO_FU_SET_RES_TREBLE_CONTROL            (0x0405)
#define USB_DEVICE_AUDIO_FU_SET_RES_GRAPHIC_EQUALIZER_CONTROL (0x0406)
#define USB_DEVICE_AUDIO_FU_SET_RES_DELAY_CONTROL             (0x0408)

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0) /* Audio 2.0 FU : the following application command value is started from ox20 */
/*! @brief Audio device class-specific ENDP SET CUR COMMAND  */
#define USB_DEVICE_AUDIO_EP_SET_CUR_PITCH_CONTROL_AUDIO20 (0x0122)
/*! @brief Audio device class-specific ENDP GET CUR COMMAND  */
#define USB_DEVICE_AUDIO_EP_GET_CUR_DATA_OVERRUN_CONTROL  (0x8121)
#define USB_DEVICE_AUDIO_EP_GET_CUR_DATA_UNDERRUN_CONTROL (0x8122)
#else
/*! @brief Audio device class-specific ENDP SET CUR COMMAND  */
#define USB_DEVICE_AUDIO_EP_SET_CUR_PITCH_CONTROL         (0x0120)
#define USB_DEVICE_AUDIO_EP_SET_CUR_SAMPLING_FREQ_CONTROL (0x0121)

/*! @brief Audio device class-specific ENDP SET MIN COMMAND  */
#define USB_DEVICE_AUDIO_EP_SET_MIN_SAMPLING_FREQ_CONTROL (0x0220)

/*! @brief Audio device class-specific ENDP SET MAX COMMAND  */
#define USB_DEVICE_AUDIO_EP_SET_MAX_SAMPLING_FREQ_CONTROL (0x0320)

/*! @brief Audio device class-specific ENDP SET RES COMMAND  */
#define USB_DEVICE_AUDIO_EP_SET_RES_SAMPLING_FREQ_CONTROL (0x0420)

/*! @brief Audio device class-specific ENDP GET CUR COMMAND  */
#define USB_DEVICE_AUDIO_EP_GET_CUR_SAMPLING_FREQ_CONTROL (0x8120)

/*! @brief Audio device class-specific ENDP GET MIN COMMAND  */
#define USB_DEVICE_AUDIO_EP_GET_MIN_SAMPLING_FREQ_CONTROL (0x8220)

/*! @brief Audio device class-specific ENDP GET MAX COMMAND  */
#define USB_DEVICE_AUDIO_EP_GET_MAX_SAMPLING_FREQ_CONTROL (0x8320)

/*! @brief Audio device class-specific ENDP GET RES COMMAND  */
#define USB_DEVICE_AUDIO_EP_GET_RES_SAMPLING_FREQ_CONTROL (0x8420)
#endif

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0) /* CS: the following application command value is started from ox30 */
/*! @brief Audio device class-specific CS SET CUR COMMAND  */
#define USB_DEVICE_AUDIO_CS_SET_CUR_SAMPLING_FREQ_CONTROL (0x0130)
#define USB_DEVICE_AUDIO_CS_SET_CUR_CLOCK_VALID_CONTROL   (0x0131)

/*! @brief Audio device class-specific CS GET CUR COMMAND  */
#define USB_DEVICE_AUDIO_CS_GET_CUR_SAMPLING_FREQ_CONTROL (0x8130)
#define USB_DEVICE_AUDIO_CS_GET_CUR_CLOCK_VALID_CONTROL   (0x8131)

/*! @brief Audio device class-specific CS GET RANGE COMMAND  */
#define USB_DEVICE_AUDIO_CS_GET_RANGE_SAMPLING_FREQ_CONTROL (0x8630)
#endif

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0) /* Audio 2.0 FU : the following application command value is started from ox40 */
/*! @brief Audio device class-specific GET RANGE COMMAND  */
#define USB_DEVICE_AUDIO_FU_GET_RANGE_VOLUME_CONTROL (0x8640)
#endif

/* Terminal : the following application command value is started from ox50 */
/*! @brief Audio device class-specific TE GET CUR COMMAND  */
#define USB_DEVICE_AUDIO_TE_GET_CUR_COPY_PROTECT_CONTROL (0x8150)
/*! @brief Audio device class-specific TE SET CUR COMMAND  */
#define USB_DEVICE_AUDIO_TE_SET_CUR_COPY_PROTECT_CONTROL (0x0150)
#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
#define USB_DEVICE_AUDIO_TE_GET_CUR_CONNECTOR_CONTROL (0x8151)
#define USB_DEVICE_AUDIO_TE_GET_CUR_OVERLOAD_CONTROL (0x8152)
#endif
/*! @}*/

/*!
 * @name USB Audio class setup request types
 * @{
 */
/*! @brief Audio device class setup request set type */
#define USB_DEVICE_AUDIO_SET_REQUEST_INTERFACE (0x21)
#define USB_DEVICE_AUDIO_SET_REQUEST_ENDPOINT  (0x22)

/*! @brief Audio device class setup request get type */
#define USB_DEVICE_AUDIO_GET_REQUEST_INTERFACE (0xA1)
#define USB_DEVICE_AUDIO_GET_REQUEST_ENDPOINT  (0xA2)
/*! @}*/

/*! @brief Available common EVENT types in audio class callback */
typedef enum
{
    kUSB_DeviceAudioEventStreamSendResponse = 0x01U, /*!< Send data completed or cancelled etc in stream pipe */
    kUSB_DeviceAudioEventStreamRecvResponse,         /*!< Data received or cancelled etc in stream pipe */
    kUSB_DeviceAudioEventControlSendResponse,        /*!< Send data completed or cancelled etc in audio control pipe */
} usb_device_audio_event_t;

/*!
 * @brief The audio device class-specific information.
 * The structure is used to pass the audio entity information filled by application.
 * Such as
 *     entity id (unit or terminal ID),
 *     entity type (unit or terminal type),
 *     and terminal type if the entity is a terminal.
 */
typedef struct _usb_device_audio_entity_struct
{
    uint8_t entityId;
    uint8_t entityType;
    uint16_t terminalType;
} usb_device_audio_entity_struct_t;

/*!
 * @brief The audio device class-specific information list.
 * The structure is used to pass the audio entity informations filled by the application.
 * The type of each entity is usb_device_audio_entity_struct_t.
 * The structure pointer is kept in the usb_device_interface_struct_t::classSpecific,
 * such as, if there are three entities (an out terminal, camera terminal, and processing unit),
 * the value of the count field is 3 and the entity field saves the every entity information.
 */
typedef struct _usb_device_audio_entities_struct
{
    usb_device_audio_entity_struct_t *entity;
    uint8_t count;
} usb_device_audio_entities_struct_t;

/*! @brief The audio device class status structure */
typedef struct _usb_device_audio_struct
{
    usb_device_handle handle;                              /*!< The device handle */
    usb_device_class_config_struct_t *configStruct;        /*!< The configuration of the class. */
    usb_device_interface_struct_t *controlInterfaceHandle; /*!< Current control interface handle */
    usb_device_interface_struct_t *streamInterfaceHandle;  /*!< Current stream interface handle */
    uint8_t configuration;                                 /*!< Current configuration */
    uint8_t controlInterfaceNumber;                        /*!< The control interface number of the class */
    uint8_t controlAlternate;                              /*!< Current alternate setting of the control interface */
    uint8_t streamInterfaceNumber;                         /*!< The stream interface number of the class */
    uint8_t streamAlternate;                               /*!< Current alternate setting of the stream interface */
    uint8_t streamInPipeBusy;                              /*!< Stream IN pipe busy flag */
    uint8_t streamOutPipeBusy;                             /*!< Stream OUT pipe busy flag */
} usb_device_audio_struct_t;

#if (USB_DEVICE_CONFIG_AUDIO_CLASS_2_0)
STRUCT_PACKED
struct _usb_device_control_range_layout3_struct
{
    uint16_t wNumSubRanges;
    uint32_t wMIN;
    uint32_t wMAX;
    uint32_t wRES;
} STRUCT_UNPACKED;
typedef struct _usb_device_control_range_layout3_struct usb_device_control_range_layout3_struct_t;
STRUCT_PACKED
struct _usb_device_control_range_layout2_struct
{
    uint16_t wNumSubRanges;
    uint16_t wMIN;
    uint16_t wMAX;
    uint16_t wRES;
} STRUCT_UNPACKED;
typedef struct _usb_device_control_range_layout2_struct usb_device_control_range_layout2_struct_t;
#endif

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @name USB Audio Class Driver
 * @{
 */
/*!
 * @brief Initializes the USB audio class.
 *
 * This function obtains a USB device handle according to the controller ID, initializes the audio class
 * with the class configuration parameters, and creates the mutex for each pipe.
 *
 * @param controllerId The ID of the controller. The value can be chosen from the kUSB_ControllerKhci0,
 *  kUSB_ControllerKhci1, kUSB_ControllerEhci0, or kUSB_ControllerEhci1.
 * @param config The user configuration structure of type usb_device_class_config_struct_t. The user
 *  populates the members of this structure and passes the pointer of this structure
 *  into this function.
 * @param handle  An out parameter. The class handle of the audio class.
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success The audio class is initialized successfully.
 * @retval kStatus_USB_Busy No audio device handle available for allocation.
 * @retval kStatus_USB_InvalidHandle The audio device handle allocation failure.
 * @retval kStatus_USB_InvalidParameter The USB device handle allocation failure.
 */
extern usb_status_t USB_DeviceAudioInit(uint8_t controllerId,
                                        usb_device_class_config_struct_t *config,
                                        class_handle_t *handle);

/*!
 * @brief Deinitializes the USB audio class.
 *
 * This function destroys the mutex for each pipe, deinitializes each endpoint of the audio class, and frees
 * the audio class handle.
 *
 * @param handle The class handle of the audio class.
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success The audio class is deinitialized successfully.
 * @retval kStatus_USB_Error The endpoint deinitialization failure.
 * @retval kStatus_USB_InvalidHandle The audio device handle or the audio class handle is invalid.
 * @retval kStatus_USB_InvalidParameter The endpoint number of the audio class handle is invalid.
 */
extern usb_status_t USB_DeviceAudioDeinit(class_handle_t handle);

/*!
 * @brief Handles the USB audio class event.
 *
 * This function responds to various events including the common device events and the class-specific events.
 * For class-specific events, it calls the class callback defined in the application to deal with the class-specific
 * event.
 *
 * @param handle The class handle of the audio class.
 * @param event The event type.
 * @param param The class handle of the audio class.
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success The audio class is deinitialized successfully.
 * @retval kStatus_USB_Error The configure structure of the audio class handle is invalid.
 * @retval kStatus_USB_InvalidHandle The audio device handle or the audio class handle is invalid.
 * @retval kStatus_USB_InvalidParameter The endpoint number of the audio class handle is invalid.
 * @retval Others The error code returned by class callback in application.
 */
extern usb_status_t USB_DeviceAudioEvent(void *handle, uint32_t event, void *param);

/*!
 * @brief Primes the endpoint to send a packet to the host.
 *
 * This function checks whether the endpoint is sending packet, then it primes the endpoint
 * with the buffer address and the buffer length if the pipe is not busy. Otherwise, it ignores this transfer by
 * returning an error code.
 *
 * @param handle The class handle of the audio class.
 * @param ep The endpoint number of the transfer.
 * @param buffer The pointer to the buffer to be transferred.
 * @param length The length of the buffer to be transferred.
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success Prime to send packet successfully.
 * @retval kStatus_USB_Busy The endpoint is busy in transferring.
 * @retval kStatus_USB_InvalidHandle The audio device handle or the audio class handle is invalid.
 * @retval kStatus_USB_ControllerNotFound The controller interface is invalid.
 *
 * @note The function can only be called in the same context.
 */
extern usb_status_t USB_DeviceAudioSend(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*!
 * @brief Primes the endpoint to receive a packet from the host.
 *
 * This function checks whether the endpoint is receiving packet, then it primes the endpoint
 * with the buffer address and the buffer length if the pipe is not busy. Otherwise, it ignores this transfer by
 * returning an error code.
 *
 * @param handle The class handle of the audio class.
 * @param ep The endpoint number of the transfer.
 * @param buffer The pointer to the buffer to be transferred.
 * @param length The length of the buffer to be transferred.
 * @return A USB error code or kStatus_USB_Success.
 * @retval kStatus_USB_Success Prime to receive packet successfully.
 * @retval kStatus_USB_Busy The endpoint is busy in transferring.
 * @retval kStatus_USB_InvalidHandle The audio device handle or the audio class handle is invalid.
 * @retval kStatus_USB_ControllerNotFound The controller interface is invalid.
 *
 * @note The function can only be called in the same context.
 */
extern usb_status_t USB_DeviceAudioRecv(class_handle_t handle, uint8_t ep, uint8_t *buffer, uint32_t length);

/*! @}*/

/*! @}*/

#endif /* __USB_DEVICE_AUDIO_H__ */
