#include "hid_gamepad.h"

#include <assert.h>
#include <inttypes.h>
#include <stddef.h>
#include <sys/types.h>

#include "util/binary.h"
#include "util/log.h"

// 2x2 bytes for left stick (X, Y)
// 3x2 bytes for right stick (Z, Rz, Rx, Ry) -- Z/Rz and Rx/Ry both carry the
// right stick values for compatibility with games reading either axis set
// 1 byte for hat switch (dpad)
// 2 bytes for buttons
// 2x2 bytes for L2/R2 triggers
#define SC_HID_GAMEPAD_EVENT_SIZE 19

// The ->buttons field stores the state for all buttons, but only some of them
// (the 16 LSB) must be transmitted "as is". The DPAD (hat switch) buttons are
// stored locally in the MSB of this field, but not transmitted as is: they are
// transformed to generate another specific byte.
#define SC_HID_BUTTONS_MASK 0xFFFF

// outside SC_HID_BUTTONS_MASK
#define SC_GAMEPAD_BUTTONS_BIT_DPAD_UP    UINT32_C(0x10000)
#define SC_GAMEPAD_BUTTONS_BIT_DPAD_DOWN  UINT32_C(0x20000)
#define SC_GAMEPAD_BUTTONS_BIT_DPAD_LEFT  UINT32_C(0x40000)
#define SC_GAMEPAD_BUTTONS_BIT_DPAD_RIGHT UINT32_C(0x80000)

/**
 * Gamepad descriptor manually crafted to transmit the input reports.
 *
 * This is the "modern" Xbox One S / Series X|S layout (as parsed by SDL's
 * Xbox One driver): the right stick is reported on Z/Rz, and the triggers on
 * the Simulation Controls usages Brake/Accelerator with a 10-bit logical
 * range. Some Android games (e.g. Honkai: Star Rail) only enable controller
 * support when the gamepad is recognized as an "Xbox Wireless Controller",
 * which relies both on the VID/PID/name triplet and on this report layout.
 *
 * The HID specification is available here:
 * <https://www.usb.org/document-library/device-class-definition-hid-111>
 *
 * The HID Usage Tables is also useful:
 * <https://www.usb.org/document-library/hid-usage-tables-15>
 */
static const uint8_t SC_HID_GAMEPAD_REPORT_DESC[] = {
    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (Gamepad)
    0x09, 0x05,

    // Collection (Application)
    0xA1, 0x01,

    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (X)   Left stick x
    0x09, 0x30,
    // Usage (Y)   Left stick y
    0x09, 0x31,
    // Usage (Z)   Right stick x
    0x09, 0x32,
    // Usage (Rz)  Right stick y
    0x09, 0x35,
    // Usage (Rx)  Right stick x (duplicated for compatibility)
    0x09, 0x33,
    // Usage (Ry)  Right stick y (duplicated for compatibility)
    0x09, 0x34,
    // Logical Minimum (0)
    0x15, 0x00,
    // Logical Maximum (65535)
    // Cannot use 26 FF FF because 0xFFFF is interpreted as signed 16-bit
    0x27, 0xFF, 0xFF, 0x00, 0x00, // little-endian
    // Report Size (16)
    0x75, 0x10,
    // Report Count (6)
    0x95, 0x06,
    // Input (Data, Variable, Absolute): 6x2 bytes (X, Y, Z, Rz, Rx, Ry)
    // The right stick is reported both on Z/Rz (modern Xbox layout) and on
    // Rx/Ry, so that games reading either axis set can use it.
    0x81, 0x02,

    // Usage Page (Generic Desktop)
    0x05, 0x01,
    // Usage (Hat switch)
    0x09, 0x39,
    // Logical Minimum (1)
    0x15, 0x01,
    // Logical Maximum (8)
    0x25, 0x08,
    // Physical Minimum (0)
    0x35, 0x00,
    // Physical Maximum (315)
    0x46, 0x3B, 0x01,
    // Unit (Degrees)
    0x65, 0x14,
    // Report Size (8)
    0x75, 0x08,
    // Report Count (1)
    0x95, 0x01,
    // Input (Data, Variable, Null State): 8-bit value (0 = neutral)
    0x81, 0x42,

    // Usage Page (Buttons)
    0x05, 0x09,
    // Usage Minimum (1)
    0x19, 0x01,
    // Usage Maximum (16)
    0x29, 0x10,
    // Logical Minimum (0)
    0x15, 0x00,
    // Logical Maximum (1)
    0x25, 0x01,
    // Report Count (16)
    0x95, 0x10,
    // Report Size (1)
    0x75, 0x01,
    // Input (Data, Variable, Absolute): 16 buttons bits
    0x81, 0x02,

    // Usage Page (Simulation Controls)
    0x05, 0x02,
    // Usage (Brake)  Left trigger
    0x0A, 0x31, 0x0C,
    // Usage (Accelerator)  Right trigger
    0x0A, 0x32, 0x0C,
    // Logical Minimum (0)
    0x15, 0x00,
    // Logical Maximum (1023)
    0x26, 0xFF, 0x03,
    // Report Size (16)
    0x75, 0x10,
    // Report Count (2)
    0x95, 0x02,
    // Input (Data, Variable, Absolute): 2x2 bytes (L2, R2)
    0x81, 0x02,

    // End Collection
    0xC0,
};

/**
 * A gamepad HID input report is 19 bytes long:
 *  - bytes 0-1:   left stick x
 *  - bytes 2-3:   left stick y
 *  - bytes 4-5:   right stick x (Z)
 *  - bytes 6-7:   right stick y (Rz)
 *  - bytes 8-9:   right stick x (Rx, duplicate)
 *  - bytes 10-11: right stick y (Ry, duplicate)
 *  - byte 12:     hat switch position (dpad)
 *  - bytes 13-14: buttons state
 *  - bytes 15-16: L2 trigger state
 *  - bytes 17-18: R2 trigger state
 *
 *                  +---------------+
 *         byte 0:  |. . . . . . . .|
 *                  |               | left stick x (0-65535, little-endian)
 *         byte 1:  |. . . . . . . .|
 *                  +---------------+
 *         byte 2:  |. . . . . . . .|
 *                  |               | left stick y (0-65535, little-endian)
 *         byte 3:  |. . . . . . . .|
 *                  +---------------+
 *         byte 4:  |. . . . . . . .|
 *                  |               | right stick x (0-65535, little-endian)
 *         byte 5:  |. . . . . . . .|
 *                  +---------------+
 *         byte 6:  |. . . . . . . .|
 *                  |               | right stick y (0-65535, little-endian)
 *         byte 7:  |. . . . . . . .|
 *                  +---------------+
 *         byte 8:  |. . . . . . . .|
 *                  |               | right stick x (Rx, duplicate)
 *         byte 9:  |. . . . . . . .|
 *                  +---------------+
 *        byte 10:  |. . . . . . . .|
 *                  |               | right stick y (Ry, duplicate)
 *        byte 11:  |. . . . . . . .|
 *                  +---------------+
 *        byte 12:  |. . . . . . . .| hat switch (dpad) position (0-8)
 *                  +---------------+
 *                     9 possible positions and their values:
 *                             8 1 2
 *                             7 0 3
 *                             6 5 4
 *                     (8 is top-left, 1 is top, 2 is top-right, etc.)
 *
 *                   ,--------------- SC_GAMEPAD_BUTTON_RIGHT_SHOULDER
 *                   | ,------------- SC_GAMEPAD_BUTTON_LEFT_SHOULDER
 *                   | |
 *                   | |   ,--------- SC_GAMEPAD_BUTTON_NORTH
 *                   | |   | ,------- SC_GAMEPAD_BUTTON_WEST
 *                   | |   | |
 *                   | |   | |   ,--- SC_GAMEPAD_BUTTON_EAST
 *                   | |   | |   | ,- SC_GAMEPAD_BUTTON_SOUTH
 *                   v v   v v   v v
 *                  +---------------+
 *        byte 13:  |. . 0 . . 0 . .|
 *                  |               | Buttons (16-bit little-endian)
 *        byte 14:  |0 . . . . . 0 0|
 *                  +---------------+
 *                     ^ ^ ^ ^ ^
 *                     | | | | |
 *                     | | | | |
 *                     | | | | `----- SC_GAMEPAD_BUTTON_BACK
 *                     | | | `------- SC_GAMEPAD_BUTTON_START
 *                     | | `--------- SC_GAMEPAD_BUTTON_GUIDE
 *                     | `----------- SC_GAMEPAD_BUTTON_LEFT_STICK
 *                     `------------- SC_GAMEPAD_BUTTON_RIGHT_STICK
 *
 *                  +---------------+
 *        byte 15:  |. . . . . . . .|
 *                  |               | L2 trigger (0-1023, little-endian)
 *        byte 16:  |0 0 0 0 0 0 . .|
 *                  +---------------+
 *        byte 17:  |. . . . . . . .|
 *                  |               | R2 trigger (0-1023, little-endian)
 *        byte 18:  |0 0 0 0 0 0 . .|
 *                  +---------------+
 */

// [-32768 to 32767] -> [0 to 65535]
#define AXIS_RESCALE(V) (uint16_t) (((int32_t) V) + 0x8000)

static void
sc_hid_gamepad_slot_init(struct sc_hid_gamepad_slot *slot,
                         uint32_t gamepad_id) {
    assert(gamepad_id != SC_GAMEPAD_ID_INVALID);
    slot->gamepad_id = gamepad_id;
    slot->buttons = 0;
    slot->axis_left_x = AXIS_RESCALE(0);
    slot->axis_left_y = AXIS_RESCALE(0);
    slot->axis_right_x = AXIS_RESCALE(0);
    slot->axis_right_y = AXIS_RESCALE(0);
    slot->axis_left_trigger = 0;
    slot->axis_right_trigger = 0;
}

static ssize_t
sc_hid_gamepad_slot_find(struct sc_hid_gamepad *hid, uint32_t gamepad_id) {
    for (size_t i = 0; i < SC_MAX_GAMEPADS; ++i) {
        if (gamepad_id == hid->slots[i].gamepad_id) {
            // found
            return i;
        }
    }

    return -1;
}

void
sc_hid_gamepad_init(struct sc_hid_gamepad *hid) {
    for (size_t i = 0; i < SC_MAX_GAMEPADS; ++i) {
        hid->slots[i].gamepad_id = SC_GAMEPAD_ID_INVALID;
    }
}

static inline uint16_t
sc_hid_gamepad_slot_get_id(size_t slot_idx) {
    assert(slot_idx < SC_MAX_GAMEPADS);
    return SC_HID_ID_GAMEPAD_FIRST + slot_idx;
}

bool
sc_hid_gamepad_generate_open(struct sc_hid_gamepad *hid,
                             struct sc_hid_open *hid_open,
                             uint32_t gamepad_id) {
    assert(gamepad_id != SC_GAMEPAD_ID_INVALID);
    ssize_t slot_idx = sc_hid_gamepad_slot_find(hid, SC_GAMEPAD_ID_INVALID);
    if (slot_idx == -1) {
        LOGW("No gamepad slot available for new gamepad %" PRIu32, gamepad_id);
        return false;
    }

    sc_hid_gamepad_slot_init(&hid->slots[slot_idx], gamepad_id);

    uint16_t hid_id = sc_hid_gamepad_slot_get_id(slot_idx);
    hid_open->hid_id = hid_id;
    hid_open->report_desc = SC_HID_GAMEPAD_REPORT_DESC;
    hid_open->report_desc_size = sizeof(SC_HID_GAMEPAD_REPORT_DESC);

    return true;
}

bool
sc_hid_gamepad_generate_close(struct sc_hid_gamepad *hid,
                              struct sc_hid_close *hid_close,
                              uint32_t gamepad_id) {
    assert(gamepad_id != SC_GAMEPAD_ID_INVALID);
    ssize_t slot_idx = sc_hid_gamepad_slot_find(hid, gamepad_id);
    if (slot_idx == -1) {
        LOGW("Unknown gamepad removed %" PRIu32, gamepad_id);
        return false;
    }

    hid->slots[slot_idx].gamepad_id = SC_GAMEPAD_ID_INVALID;

    uint16_t hid_id = sc_hid_gamepad_slot_get_id(slot_idx);
    hid_close->hid_id = hid_id;

    return true;
}

static uint8_t
sc_hid_gamepad_get_dpad_value(uint32_t buttons) {
    // Value depending on direction:
    //     8 1 2
    //     7 0 3
    //     6 5 4
    if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_UP) {
        if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_LEFT) {
            return 8;
        }
        if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_RIGHT) {
            return 2;
        }
        return 1;
    }
    if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_DOWN) {
        if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_LEFT) {
            return 6;
        }
        if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_RIGHT) {
            return 4;
        }
        return 5;
    }
    if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_LEFT) {
        return 7;
    }
    if (buttons & SC_GAMEPAD_BUTTONS_BIT_DPAD_RIGHT) {
        return 3;
    }

    return 0;
}

static void
sc_hid_gamepad_event_from_slot(uint16_t hid_id,
                               const struct sc_hid_gamepad_slot *slot,
                               struct sc_hid_input *hid_input) {
    hid_input->hid_id = hid_id;
    hid_input->size = SC_HID_GAMEPAD_EVENT_SIZE;

    uint8_t *data = hid_input->data;
    // Values must be written in little-endian
    sc_write16le(data, slot->axis_left_x);
    sc_write16le(data + 2, slot->axis_left_y);
    sc_write16le(data + 4, slot->axis_right_x);
    sc_write16le(data + 6, slot->axis_right_y);
    // Duplicate the right stick on Rx/Ry, for games reading these axes
    // instead of Z/Rz
    sc_write16le(data + 8, slot->axis_right_x);
    sc_write16le(data + 10, slot->axis_right_y);
    data[12] = sc_hid_gamepad_get_dpad_value(slot->buttons);
    sc_write16le(data + 13, slot->buttons & SC_HID_BUTTONS_MASK);
    // Triggers are scaled from [0, 32767] to [0, 1023] (10-bit, like a real
    // Xbox Wireless Controller)
    sc_write16le(data + 15, slot->axis_left_trigger >> 5);
    sc_write16le(data + 17, slot->axis_right_trigger >> 5);
}

static uint32_t
sc_hid_gamepad_get_button_id(enum sc_gamepad_button button) {
    switch (button) {
        case SC_GAMEPAD_BUTTON_SOUTH:
            return 0x0001;
        case SC_GAMEPAD_BUTTON_EAST:
            return 0x0002;
        case SC_GAMEPAD_BUTTON_WEST:
            return 0x0008;
        case SC_GAMEPAD_BUTTON_NORTH:
            return 0x0010;
        case SC_GAMEPAD_BUTTON_BACK:
            return 0x0400;
        case SC_GAMEPAD_BUTTON_GUIDE:
            return 0x1000;
        case SC_GAMEPAD_BUTTON_START:
            return 0x0800;
        case SC_GAMEPAD_BUTTON_LEFT_STICK:
            return 0x2000;
        case SC_GAMEPAD_BUTTON_RIGHT_STICK:
            return 0x4000;
        case SC_GAMEPAD_BUTTON_LEFT_SHOULDER:
            return 0x0040;
        case SC_GAMEPAD_BUTTON_RIGHT_SHOULDER:
            return 0x0080;
        case SC_GAMEPAD_BUTTON_DPAD_UP:
            return SC_GAMEPAD_BUTTONS_BIT_DPAD_UP;
        case SC_GAMEPAD_BUTTON_DPAD_DOWN:
            return SC_GAMEPAD_BUTTONS_BIT_DPAD_DOWN;
        case SC_GAMEPAD_BUTTON_DPAD_LEFT:
            return SC_GAMEPAD_BUTTONS_BIT_DPAD_LEFT;
        case SC_GAMEPAD_BUTTON_DPAD_RIGHT:
            return SC_GAMEPAD_BUTTONS_BIT_DPAD_RIGHT;
        default:
            // unknown button, ignore
            return 0;
    }
}

bool
sc_hid_gamepad_generate_input_from_button(struct sc_hid_gamepad *hid,
                                          struct sc_hid_input *hid_input,
                                const struct sc_gamepad_button_event *event) {
    if ((event->button < 0) || (event->button > 15)) {
        return false;
    }

    uint32_t gamepad_id = event->gamepad_id;

    ssize_t slot_idx = sc_hid_gamepad_slot_find(hid, gamepad_id);
    if (slot_idx == -1) {
        LOGW("Axis event for unknown gamepad %" PRIu32, gamepad_id);
        return false;
    }

    assert(slot_idx < SC_MAX_GAMEPADS);

    struct sc_hid_gamepad_slot *slot = &hid->slots[slot_idx];

    uint32_t button = sc_hid_gamepad_get_button_id(event->button);
    if (!button) {
        // unknown button, ignore
        return false;
    }

    if (event->action == SC_ACTION_DOWN) {
        slot->buttons |= button;
    } else {
        assert(event->action == SC_ACTION_UP);
        slot->buttons &= ~button;
    }

    uint16_t hid_id = sc_hid_gamepad_slot_get_id(slot_idx);
    sc_hid_gamepad_event_from_slot(hid_id, slot, hid_input);

    return true;
}

bool
sc_hid_gamepad_generate_input_from_axis(struct sc_hid_gamepad *hid,
                                        struct sc_hid_input *hid_input,
                                const struct sc_gamepad_axis_event *event) {
    uint32_t gamepad_id = event->gamepad_id;

    ssize_t slot_idx = sc_hid_gamepad_slot_find(hid, gamepad_id);
    if (slot_idx == -1) {
        LOGW("Button event for unknown gamepad %" PRIu32, gamepad_id);
        return false;
    }

    assert(slot_idx < SC_MAX_GAMEPADS);

    struct sc_hid_gamepad_slot *slot = &hid->slots[slot_idx];

    switch (event->axis) {
        case SC_GAMEPAD_AXIS_LEFTX:
            slot->axis_left_x = AXIS_RESCALE(event->value);
            break;
        case SC_GAMEPAD_AXIS_LEFTY:
            slot->axis_left_y = AXIS_RESCALE(event->value);
            break;
        case SC_GAMEPAD_AXIS_RIGHTX:
            slot->axis_right_x = AXIS_RESCALE(event->value);
            break;
        case SC_GAMEPAD_AXIS_RIGHTY:
            slot->axis_right_y = AXIS_RESCALE(event->value);
            break;
        case SC_GAMEPAD_AXIS_LEFT_TRIGGER:
            // Trigger is always positive between 0 and 32767
            slot->axis_left_trigger = MAX(0, event->value);
            break;
        case SC_GAMEPAD_AXIS_RIGHT_TRIGGER:
            // Trigger is always positive between 0 and 32767
            slot->axis_right_trigger = MAX(0, event->value);
            break;
        default:
            return false;
    }

    uint16_t hid_id = sc_hid_gamepad_slot_get_id(slot_idx);
    sc_hid_gamepad_event_from_slot(hid_id, slot, hid_input);

    return true;
}
