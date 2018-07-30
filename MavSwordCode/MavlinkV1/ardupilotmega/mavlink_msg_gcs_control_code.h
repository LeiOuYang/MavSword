#pragma once
// MESSAGE GCS_CONTROL_CODE PACKING

#define MAVLINK_MSG_ID_GCS_CONTROL_CODE 238

MAVPACKED(
typedef struct __mavlink_gcs_control_code_t {
 int8_t reservation; /*< reservatation byte*/
 int8_t device; /*< device control id and param id*/
 int8_t controlValue; /*< control value*/
}) mavlink_gcs_control_code_t;

#define MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN 3
#define MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN 3
#define MAVLINK_MSG_ID_238_LEN 3
#define MAVLINK_MSG_ID_238_MIN_LEN 3

#define MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC 123
#define MAVLINK_MSG_ID_238_CRC 123



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GCS_CONTROL_CODE { \
    238, \
    "GCS_CONTROL_CODE", \
    3, \
    {  { "reservation", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_gcs_control_code_t, reservation) }, \
         { "device", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_gcs_control_code_t, device) }, \
         { "controlValue", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_gcs_control_code_t, controlValue) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GCS_CONTROL_CODE { \
    "GCS_CONTROL_CODE", \
    3, \
    {  { "reservation", NULL, MAVLINK_TYPE_INT8_T, 0, 0, offsetof(mavlink_gcs_control_code_t, reservation) }, \
         { "device", NULL, MAVLINK_TYPE_INT8_T, 0, 1, offsetof(mavlink_gcs_control_code_t, device) }, \
         { "controlValue", NULL, MAVLINK_TYPE_INT8_T, 0, 2, offsetof(mavlink_gcs_control_code_t, controlValue) }, \
         } \
}
#endif

/**
 * @brief Pack a gcs_control_code message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param reservation reservatation byte
 * @param device device control id and param id
 * @param controlValue control value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_control_code_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int8_t reservation, int8_t device, int8_t controlValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN];
    _mav_put_int8_t(buf, 0, reservation);
    _mav_put_int8_t(buf, 1, device);
    _mav_put_int8_t(buf, 2, controlValue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN);
#else
    mavlink_gcs_control_code_t packet;
    packet.reservation = reservation;
    packet.device = device;
    packet.controlValue = controlValue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_CONTROL_CODE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
}

/**
 * @brief Pack a gcs_control_code message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param reservation reservatation byte
 * @param device device control id and param id
 * @param controlValue control value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gcs_control_code_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int8_t reservation,int8_t device,int8_t controlValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN];
    _mav_put_int8_t(buf, 0, reservation);
    _mav_put_int8_t(buf, 1, device);
    _mav_put_int8_t(buf, 2, controlValue);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN);
#else
    mavlink_gcs_control_code_t packet;
    packet.reservation = reservation;
    packet.device = device;
    packet.controlValue = controlValue;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GCS_CONTROL_CODE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
}

/**
 * @brief Encode a gcs_control_code struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gcs_control_code C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_control_code_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gcs_control_code_t* gcs_control_code)
{
    return mavlink_msg_gcs_control_code_pack(system_id, component_id, msg, gcs_control_code->reservation, gcs_control_code->device, gcs_control_code->controlValue);
}

/**
 * @brief Encode a gcs_control_code struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gcs_control_code C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gcs_control_code_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gcs_control_code_t* gcs_control_code)
{
    return mavlink_msg_gcs_control_code_pack_chan(system_id, component_id, chan, msg, gcs_control_code->reservation, gcs_control_code->device, gcs_control_code->controlValue);
}

/**
 * @brief Send a gcs_control_code message
 * @param chan MAVLink channel to send the message
 *
 * @param reservation reservatation byte
 * @param device device control id and param id
 * @param controlValue control value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gcs_control_code_send(mavlink_channel_t chan, int8_t reservation, int8_t device, int8_t controlValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN];
    _mav_put_int8_t(buf, 0, reservation);
    _mav_put_int8_t(buf, 1, device);
    _mav_put_int8_t(buf, 2, controlValue);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_CONTROL_CODE, buf, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
#else
    mavlink_gcs_control_code_t packet;
    packet.reservation = reservation;
    packet.device = device;
    packet.controlValue = controlValue;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_CONTROL_CODE, (const char *)&packet, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
#endif
}

/**
 * @brief Send a gcs_control_code message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gcs_control_code_send_struct(mavlink_channel_t chan, const mavlink_gcs_control_code_t* gcs_control_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gcs_control_code_send(chan, gcs_control_code->reservation, gcs_control_code->device, gcs_control_code->controlValue);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_CONTROL_CODE, (const char *)gcs_control_code, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
#endif
}

#if MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gcs_control_code_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int8_t reservation, int8_t device, int8_t controlValue)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int8_t(buf, 0, reservation);
    _mav_put_int8_t(buf, 1, device);
    _mav_put_int8_t(buf, 2, controlValue);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_CONTROL_CODE, buf, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
#else
    mavlink_gcs_control_code_t *packet = (mavlink_gcs_control_code_t *)msgbuf;
    packet->reservation = reservation;
    packet->device = device;
    packet->controlValue = controlValue;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GCS_CONTROL_CODE, (const char *)packet, MAVLINK_MSG_ID_GCS_CONTROL_CODE_MIN_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN, MAVLINK_MSG_ID_GCS_CONTROL_CODE_CRC);
#endif
}
#endif

#endif

// MESSAGE GCS_CONTROL_CODE UNPACKING


/**
 * @brief Get field reservation from gcs_control_code message
 *
 * @return reservatation byte
 */
static inline int8_t mavlink_msg_gcs_control_code_get_reservation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  0);
}

/**
 * @brief Get field device from gcs_control_code message
 *
 * @return device control id and param id
 */
static inline int8_t mavlink_msg_gcs_control_code_get_device(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  1);
}

/**
 * @brief Get field controlValue from gcs_control_code message
 *
 * @return control value
 */
static inline int8_t mavlink_msg_gcs_control_code_get_controlValue(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  2);
}

/**
 * @brief Decode a gcs_control_code message into a struct
 *
 * @param msg The message to decode
 * @param gcs_control_code C-struct to decode the message contents into
 */
static inline void mavlink_msg_gcs_control_code_decode(const mavlink_message_t* msg, mavlink_gcs_control_code_t* gcs_control_code)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gcs_control_code->reservation = mavlink_msg_gcs_control_code_get_reservation(msg);
    gcs_control_code->device = mavlink_msg_gcs_control_code_get_device(msg);
    gcs_control_code->controlValue = mavlink_msg_gcs_control_code_get_controlValue(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN? msg->len : MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN;
        memset(gcs_control_code, 0, MAVLINK_MSG_ID_GCS_CONTROL_CODE_LEN);
    memcpy(gcs_control_code, _MAV_PAYLOAD(msg), len);
#endif
}
