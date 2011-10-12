/*
 * keypad_ma87712_crespo.h
 *
 */
#ifndef __KEYPAD_MA87712_CRESPO_H
#define __KEYPAD_MA87712_CRESPO_H

unsigned char ma87712kbd_keycode[0x6D] = {	/* ma87712 keyboard layout */
/* 0x01 */	[1] = KEY_ESC,
/* 0x02 */	[2] = KEY_1,
/* 0x03 */	[3] = KEY_2,
/* 0x04 */	[4] = KEY_3,
/* 0x05 */	[5] = KEY_4,
/* 0x06 */	[6] = KEY_5,
/* 0x07 */	[7] = KEY_6,
/* 0x08 */	[8] = KEY_7,
/* 0x09 */	[9] = KEY_8,
/* 0x0a */	[10] = KEY_9,
/* 0x0b */	[11] = KEY_0,
/* 0x0c */	[12] = KEY_MINUS,
/* 0x0d */	[13] = KEY_EQUAL,
/* 0x0e */	[14] = KEY_BACKSPACE,
/* 0x0f */	[15] = KEY_TAB,
/* 0x10 */	[16] = KEY_Q,
/* 0x11 */	[17] = KEY_W,
/* 0x12 */	[18] = KEY_E,
/* 0x13 */	[19] = KEY_R,
/* 0x14 */	[20] = KEY_T,
/* 0x15 */	[21] = KEY_Y,
/* 0x16 */	[22] = KEY_U,
/* 0x17 */	[23] = KEY_I,
/* 0x18 */	[24] = KEY_O,
/* 0x19 */	[25] = KEY_P,
/* 0x1a */	[26] = KEY_LEFTBRACE,
/* 0x1b */	[27] = KEY_RIGHTBRACE,
/* 0x1c */	[28] = KEY_ENTER,
/* 0x1d */	[29] = KEY_LEFTCTRL,
/* 0x1e */	[30] = KEY_A,
/* 0x1f */	[31] = KEY_S,
/* 0x20 */	[32] = KEY_D,
/* 0x21 */	[33] = KEY_F,
/* 0x22 */	[34] = KEY_G,
/* 0x23 */	[35] = KEY_H,
/* 0x24 */	[36] = KEY_J,
/* 0x25 */	[37] = KEY_K,
/* 0x26 */	[38] = KEY_L,
/* 0x27 */	[39] = KEY_SEMICOLON,
/* 0x28 */	[40] = KEY_APOSTROPHE,
/* 0x29 */	[41] = KEY_GRAVE,	/* FIXME */
/* 0x2a */	[42] = KEY_LEFTSHIFT,
/* 0x2b */	[43] = KEY_BACKSLASH,
/* 0x2c */	[44] = KEY_Z,
/* 0x2d */	[45] = KEY_X,
/* 0x2e */	[46] = KEY_C,
/* 0x2f */	[47] = KEY_V,
/* 0x30 */	[48] = KEY_B,
/* 0x31 */	[49] = KEY_N,
/* 0x32 */	[50] = KEY_M,
/* 0x33 */	[51] = KEY_COMMA,
/* 0x34 */	[52] = KEY_DOT,
/* 0x35 */	[53] = KEY_SLASH,
/* 0x36 */	[54] = KEY_RIGHTSHIFT,
/* 0x38 */	[56] = KEY_LEFTALT,
/* 0x39 */	[57] = KEY_SPACE,
/* 0x3a */	[58] = KEY_CAPSLOCK,
/* 0x3b */	[59] = KEY_F1,
/* 0x3c */	[60] = KEY_F2,
/* 0x3d */	[61] = KEY_F3,
/* 0x3e */	[62] = KEY_F4,
/* 0x3f */	[63] = KEY_F5,
/* 0x40 */	[64] = KEY_F6,
/* 0x41 */	[65] = KEY_F7,
/* 0x42 */	[66] = KEY_F8,
/* 0x44 */	[68] = KEY_F10,
/* 0x57 */	[87] = KEY_F11,
/* 0x5b */	[91] = KEY_HIRAGANA,	/* FIXME */
/* 0x5c */	[92] = KEY_HENKAN,		/* FIXME */
/* 0x61 */	[97] = KEY_RIGHTCTRL,
/* 0x64 */	[100] = KEY_KPRIGHTPAREN,	/* FIXME */
/* 0x67 */	[103] = KEY_UP,
/* 0x69 */	[105] = KEY_LEFT,
/* 0x6a */	[106] = KEY_RIGHT,
/* 0x6c */	[108] = KEY_DOWN,
};
///* 0x7d */	[125] = ?	/* FIXME */
///* 0xe5 */	[229] = ?	/* FIXME */
///* 0x153 */	[339] = ?	/* FIXME */

#define KEYPAD_MA87712_KEYCODE		ma87712kbd_keycode
#define KEYPAD_MA87712_KEYCODEMAX	ARRAY_SIZE(ma87712kbd_keycode)
#define KEYPAD_MA87712_KEYCODESIZE	sizeof(unsigned char)

#endif
