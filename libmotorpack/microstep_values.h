/*
 * microstep_values.h
 *
 *  Created on: May 1, 2016
 *      Author: Dean Miller
 */

#ifndef MICROSTEP_VALUES_H_
#define MICROSTEP_VALUES_H_

/* 1/256 microstepping possible with this sin table
 *
 * |	bits 0-7	|	BVREF
 * |	bits 8-15	|	AVREF
 * |	bit 16		|	BPH
 * |	bit 17		|	APH
 */

uint32_t microstep_values[1023] = {
		0x200ff,
		0x201fe,
		0x203fe,
		0x204fe,
		0x206fe,
		0x207fe,
		0x209fe,
		0x20afe,
		0x20cfe,
		0x20efe,
		0x20ffe,
		0x211fe,
		0x212fe,
		0x214fe,
		0x215fe,
		0x217fd,
		0x219fd,
		0x21afd,
		0x21cfd,
		0x21dfd,
		0x21ffd,
		0x220fc,
		0x222fc,
		0x223fc,
		0x225fc,
		0x227fb,
		0x228fb,
		0x22afb,
		0x22bfb,
		0x22dfa,
		0x22efa,
		0x230fa,
		0x231fa,
		0x233f9,
		0x234f9,
		0x236f9,
		0x237f8,
		0x239f8,
		0x23af8,
		0x23cf7,
		0x23ef7,
		0x23ff6,
		0x241f6,
		0x242f6,
		0x244f5,
		0x245f5,
		0x247f4,
		0x248f4,
		0x24af3,
		0x24bf3,
		0x24df3,
		0x24ef2,
		0x250f2,
		0x251f1,
		0x253f1,
		0x254f0,
		0x255f0,
		0x257ef,
		0x258ee,
		0x25aee,
		0x25bed,
		0x25ded,
		0x25eec,
		0x260ec,
		0x261eb,
		0x263ea,
		0x264ea,
		0x265e9,
		0x267e9,
		0x268e8,
		0x26ae7,
		0x26be7,
		0x26de6,
		0x26ee5,
		0x26fe5,
		0x271e4,
		0x272e3,
		0x274e3,
		0x275e2,
		0x276e1,
		0x278e0,
		0x279e0,
		0x27bdf,
		0x27cde,
		0x27ddd,
		0x27fdd,
		0x280dc,
		0x281db,
		0x283da,
		0x284d9,
		0x285d9,
		0x287d8,
		0x288d7,
		0x289d6,
		0x28bd5,
		0x28cd4,
		0x28dd3,
		0x28fd3,
		0x290d2,
		0x291d1,
		0x292d0,
		0x294cf,
		0x295ce,
		0x296cd,
		0x298cc,
		0x299cb,
		0x29aca,
		0x29bc9,
		0x29dc8,
		0x29ec7,
		0x29fc6,
		0x2a0c5,
		0x2a1c5,
		0x2a3c4,
		0x2a4c3,
		0x2a5c1,
		0x2a6c0,
		0x2a7bf,
		0x2a9be,
		0x2aabd,
		0x2abbc,
		0x2acbb,
		0x2adba,
		0x2aeb9,
		0x2afb8,
		0x2b1b7,
		0x2b2b6,
		0x2b3b5,
		0x2b4b4,
		0x2b5b3,
		0x2b6b1,
		0x2b7b0,
		0x2b8af,
		0x2b9ae,
		0x2baad,
		0x2bcac,
		0x2bdab,
		0x2bea9,
		0x2bfa8,
		0x2c0a7,
		0x2c1a6,
		0x2c2a5,
		0x2c3a4,
		0x2c4a2,
		0x2c5a1,
		0x2c6a0,
		0x2c79f,
		0x2c89d,
		0x2c99c,
		0x2ca9b,
		0x2cb9a,
		0x2cc98,
		0x2cc97,
		0x2cd96,
		0x2ce95,
		0x2cf93,
		0x2d092,
		0x2d191,
		0x2d290,
		0x2d38e,
		0x2d48d,
		0x2d58c,
		0x2d58a,
		0x2d689,
		0x2d788,
		0x2d886,
		0x2d985,
		0x2da84,
		0x2da82,
		0x2db81,
		0x2dc80,
		0x2dd7e,
		0x2de7d,
		0x2de7c,
		0x2df7a,
		0x2e079,
		0x2e177,
		0x2e176,
		0x2e275,
		0x2e373,
		0x2e372,
		0x2e471,
		0x2e56f,
		0x2e56e,
		0x2e66c,
		0x2e76b,
		0x2e769,
		0x2e868,
		0x2e967,
		0x2e965,
		0x2ea64,
		0x2eb62,
		0x2eb61,
		0x2ec5f,
		0x2ec5e,
		0x2ed5c,
		0x2ee5b,
		0x2ee5a,
		0x2ef58,
		0x2ef57,
		0x2f055,
		0x2f054,
		0x2f152,
		0x2f151,
		0x2f24f,
		0x2f24e,
		0x2f34c,
		0x2f34b,
		0x2f449,
		0x2f448,
		0x2f446,
		0x2f545,
		0x2f543,
		0x2f642,
		0x2f640,
		0x2f73f,
		0x2f73d,
		0x2f73c,
		0x2f83a,
		0x2f839,
		0x2f837,
		0x2f936,
		0x2f934,
		0x2f932,
		0x2fa31,
		0x2fa2f,
		0x2fa2e,
		0x2fb2c,
		0x2fb2b,
		0x2fb29,
		0x2fb28,
		0x2fc26,
		0x2fc25,
		0x2fc23,
		0x2fc21,
		0x2fc20,
		0x2fd1e,
		0x2fd1d,
		0x2fd1b,
		0x2fd1a,
		0x2fd18,
		0x2fd17,
		0x2fe15,
		0x2fe13,
		0x2fe12,
		0x2fe10,
		0x2fe0f,
		0x2fe0d,
		0x2fe0c,
		0x2fe0a,
		0x2fe09,
		0x2fe07,
		0x2fe05,
		0x2fe04,
		0x2fe02,
		0x2fe01,
		0x2fe00,
		0x3fe01,
		0x3fe03,
		0x3fe05,
		0x3fe06,
		0x3fe08,
		0x3fe09,
		0x3fe0b,
		0x3fe0c,
		0x3fe0e,
		0x3fe10,
		0x3fe11,
		0x3fe13,
		0x3fe14,
		0x3fe16,
		0x3fd17,
		0x3fd19,
		0x3fd1a,
		0x3fd1c,
		0x3fd1e,
		0x3fd1f,
		0x3fc21,
		0x3fc22,
		0x3fc24,
		0x3fc25,
		0x3fb27,
		0x3fb28,
		0x3fb2a,
		0x3fb2c,
		0x3fa2d,
		0x3fa2f,
		0x3fa30,
		0x3fa32,
		0x3f933,
		0x3f935,
		0x3f936,
		0x3f838,
		0x3f839,
		0x3f73b,
		0x3f73c,
		0x3f73e,
		0x3f63f,
		0x3f641,
		0x3f642,
		0x3f544,
		0x3f545,
		0x3f447,
		0x3f448,
		0x3f34a,
		0x3f34b,
		0x3f24d,
		0x3f24e,
		0x3f150,
		0x3f151,
		0x3f053,
		0x3f054,
		0x3ef56,
		0x3ef57,
		0x3ee59,
		0x3ee5a,
		0x3ed5c,
		0x3ed5d,
		0x3ec5f,
		0x3eb60,
		0x3eb62,
		0x3ea63,
		0x3ea64,
		0x3e966,
		0x3e867,
		0x3e869,
		0x3e76a,
		0x3e66c,
		0x3e66d,
		0x3e56e,
		0x3e470,
		0x3e471,
		0x3e373,
		0x3e274,
		0x3e275,
		0x3e177,
		0x3e078,
		0x3df7a,
		0x3df7b,
		0x3de7c,
		0x3dd7e,
		0x3dc7f,
		0x3dc80,
		0x3db82,
		0x3da83,
		0x3d984,
		0x3d886,
		0x3d787,
		0x3d788,
		0x3d68a,
		0x3d58b,
		0x3d48c,
		0x3d38e,
		0x3d28f,
		0x3d190,
		0x3d191,
		0x3d093,
		0x3cf94,
		0x3ce95,
		0x3cd97,
		0x3cc98,
		0x3cb99,
		0x3ca9a,
		0x3c99c,
		0x3c89d,
		0x3c79e,
		0x3c69f,
		0x3c5a0,
		0x3c4a2,
		0x3c3a3,
		0x3c2a4,
		0x3c1a5,
		0x3c0a6,
		0x3bfa8,
		0x3bea9,
		0x3bdaa,
		0x3bcab,
		0x3bbac,
		0x3baad,
		0x3b9af,
		0x3b8b0,
		0x3b7b1,
		0x3b6b2,
		0x3b5b3,
		0x3b3b4,
		0x3b2b5,
		0x3b1b6,
		0x3b0b8,
		0x3afb9,
		0x3aeba,
		0x3adbb,
		0x3abbc,
		0x3aabd,
		0x3a9be,
		0x3a8bf,
		0x3a7c0,
		0x3a6c1,
		0x3a4c2,
		0x3a3c3,
		0x3a2c4,
		0x3a1c5,
		0x3a0c6,
		0x39ec7,
		0x39dc8,
		0x39cc9,
		0x39bca,
		0x399cb,
		0x398cc,
		0x397cd,
		0x396ce,
		0x394cf,
		0x393cf,
		0x392d0,
		0x391d1,
		0x38fd2,
		0x38ed3,
		0x38dd4,
		0x38bd5,
		0x38ad6,
		0x389d6,
		0x387d7,
		0x386d8,
		0x385d9,
		0x383da,
		0x382db,
		0x381db,
		0x37fdc,
		0x37edd,
		0x37dde,
		0x37bde,
		0x37adf,
		0x379e0,
		0x377e1,
		0x376e1,
		0x374e2,
		0x373e3,
		0x372e4,
		0x370e4,
		0x36fe5,
		0x36de6,
		0x36ce6,
		0x36ae7,
		0x369e8,
		0x368e8,
		0x366e9,
		0x365ea,
		0x363ea,
		0x362eb,
		0x360eb,
		0x35fec,
		0x35eed,
		0x35ced,
		0x35bee,
		0x359ee,
		0x358ef,
		0x356ef,
		0x355f0,
		0x353f0,
		0x352f1,
		0x350f1,
		0x34ff2,
		0x34df2,
		0x34cf3,
		0x34af3,
		0x349f4,
		0x347f4,
		0x346f5,
		0x344f5,
		0x343f5,
		0x341f6,
		0x340f6,
		0x33ef7,
		0x33df7,
		0x33bf7,
		0x33af8,
		0x338f8,
		0x337f8,
		0x335f9,
		0x334f9,
		0x332f9,
		0x331fa,
		0x32ffa,
		0x32dfa,
		0x32cfb,
		0x32afb,
		0x329fb,
		0x327fb,
		0x326fc,
		0x324fc,
		0x323fc,
		0x321fc,
		0x320fc,
		0x31efd,
		0x31cfd,
		0x31bfd,
		0x319fd,
		0x318fd,
		0x316fd,
		0x315fe,
		0x313fe,
		0x311fe,
		0x310fe,
		0x30efe,
		0x30dfe,
		0x30bfe,
		0x30afe,
		0x308fe,
		0x307fe,
		0x305fe,
		0x303fe,
		0x302fe,
		0x300fe,
		0x300fe,
		0x102fe,
		0x103fe,
		0x105fe,
		0x107fe,
		0x108fe,
		0x10afe,
		0x10bfe,
		0x10dfe,
		0x10efe,
		0x110fe,
		0x111fe,
		0x113fe,
		0x115fe,
		0x116fd,
		0x118fd,
		0x119fd,
		0x11bfd,
		0x11cfd,
		0x11efd,
		0x120fc,
		0x121fc,
		0x123fc,
		0x124fc,
		0x126fc,
		0x127fb,
		0x129fb,
		0x12afb,
		0x12cfb,
		0x12dfa,
		0x12ffa,
		0x131fa,
		0x132f9,
		0x134f9,
		0x135f9,
		0x137f8,
		0x138f8,
		0x13af8,
		0x13bf7,
		0x13df7,
		0x13ef7,
		0x140f6,
		0x141f6,
		0x143f5,
		0x144f5,
		0x146f5,
		0x147f4,
		0x149f4,
		0x14af3,
		0x14cf3,
		0x14df2,
		0x14ff2,
		0x150f1,
		0x152f1,
		0x153f0,
		0x155f0,
		0x156ef,
		0x158ef,
		0x159ee,
		0x15bee,
		0x15ced,
		0x15eed,
		0x15fec,
		0x160eb,
		0x162eb,
		0x163ea,
		0x165ea,
		0x166e9,
		0x168e8,
		0x169e8,
		0x16ae7,
		0x16ce6,
		0x16de6,
		0x16fe5,
		0x170e4,
		0x172e4,
		0x173e3,
		0x174e2,
		0x176e1,
		0x177e1,
		0x179e0,
		0x17adf,
		0x17bde,
		0x17dde,
		0x17edd,
		0x17fdc,
		0x181db,
		0x182db,
		0x183da,
		0x185d9,
		0x186d8,
		0x187d7,
		0x189d6,
		0x18ad6,
		0x18bd5,
		0x18dd4,
		0x18ed3,
		0x18fd2,
		0x191d1,
		0x192d0,
		0x193cf,
		0x194cf,
		0x196ce,
		0x197cd,
		0x198cc,
		0x199cb,
		0x19bca,
		0x19cc9,
		0x19dc8,
		0x19ec7,
		0x1a0c6,
		0x1a1c5,
		0x1a2c4,
		0x1a3c3,
		0x1a4c2,
		0x1a6c1,
		0x1a7c0,
		0x1a8bf,
		0x1a9be,
		0x1aabd,
		0x1abbc,
		0x1adbb,
		0x1aeba,
		0x1afb9,
		0x1b0b8,
		0x1b1b6,
		0x1b2b5,
		0x1b3b4,
		0x1b5b3,
		0x1b6b2,
		0x1b7b1,
		0x1b8b0,
		0x1b9af,
		0x1baad,
		0x1bbac,
		0x1bcab,
		0x1bdaa,
		0x1bea9,
		0x1bfa8,
		0x1c0a6,
		0x1c1a5,
		0x1c2a4,
		0x1c3a3,
		0x1c4a2,
		0x1c5a0,
		0x1c69f,
		0x1c79e,
		0x1c89d,
		0x1c99c,
		0x1ca9a,
		0x1cb99,
		0x1cc98,
		0x1cd97,
		0x1ce95,
		0x1cf94,
		0x1d093,
		0x1d191,
		0x1d190,
		0x1d28f,
		0x1d38e,
		0x1d48c,
		0x1d58b,
		0x1d68a,
		0x1d788,
		0x1d787,
		0x1d886,
		0x1d984,
		0x1da83,
		0x1db82,
		0x1dc80,
		0x1dc7f,
		0x1dd7e,
		0x1de7c,
		0x1df7b,
		0x1df7a,
		0x1e078,
		0x1e177,
		0x1e275,
		0x1e274,
		0x1e373,
		0x1e471,
		0x1e470,
		0x1e56e,
		0x1e66d,
		0x1e66c,
		0x1e76a,
		0x1e869,
		0x1e867,
		0x1e966,
		0x1ea64,
		0x1ea63,
		0x1eb62,
		0x1eb60,
		0x1ec5f,
		0x1ed5d,
		0x1ed5c,
		0x1ee5a,
		0x1ee59,
		0x1ef57,
		0x1ef56,
		0x1f054,
		0x1f053,
		0x1f151,
		0x1f150,
		0x1f24e,
		0x1f24d,
		0x1f34b,
		0x1f34a,
		0x1f448,
		0x1f447,
		0x1f545,
		0x1f544,
		0x1f642,
		0x1f641,
		0x1f63f,
		0x1f73e,
		0x1f73c,
		0x1f73b,
		0x1f839,
		0x1f838,
		0x1f936,
		0x1f935,
		0x1f933,
		0x1fa32,
		0x1fa30,
		0x1fa2f,
		0x1fa2d,
		0x1fb2c,
		0x1fb2a,
		0x1fb28,
		0x1fb27,
		0x1fc25,
		0x1fc24,
		0x1fc22,
		0x1fc21,
		0x1fd1f,
		0x1fd1e,
		0x1fd1c,
		0x1fd1a,
		0x1fd19,
		0x1fd17,
		0x1fe16,
		0x1fe14,
		0x1fe13,
		0x1fe11,
		0x1fe10,
		0x1fe0e,
		0x1fe0c,
		0x1fe0b,
		0x1fe09,
		0x1fe08,
		0x1fe06,
		0x1fe05,
		0x1fe03,
		0x1fe01,
		0x1fe00,
		0xfe01,
		0xfe02,
		0xfe04,
		0xfe05,
		0xfe07,
		0xfe09,
		0xfe0a,
		0xfe0c,
		0xfe0d,
		0xfe0f,
		0xfe10,
		0xfe12,
		0xfe13,
		0xfe15,
		0xfd17,
		0xfd18,
		0xfd1a,
		0xfd1b,
		0xfd1d,
		0xfd1e,
		0xfc20,
		0xfc21,
		0xfc23,
		0xfc25,
		0xfc26,
		0xfb28,
		0xfb29,
		0xfb2b,
		0xfb2c,
		0xfa2e,
		0xfa2f,
		0xfa31,
		0xf932,
		0xf934,
		0xf936,
		0xf837,
		0xf839,
		0xf83a,
		0xf73c,
		0xf73d,
		0xf73f,
		0xf640,
		0xf642,
		0xf543,
		0xf545,
		0xf446,
		0xf448,
		0xf449,
		0xf34b,
		0xf34c,
		0xf24e,
		0xf24f,
		0xf151,
		0xf152,
		0xf054,
		0xf055,
		0xef57,
		0xef58,
		0xee5a,
		0xee5b,
		0xed5c,
		0xec5e,
		0xec5f,
		0xeb61,
		0xeb62,
		0xea64,
		0xe965,
		0xe967,
		0xe868,
		0xe769,
		0xe76b,
		0xe66c,
		0xe56e,
		0xe56f,
		0xe471,
		0xe372,
		0xe373,
		0xe275,
		0xe176,
		0xe177,
		0xe079,
		0xdf7a,
		0xde7c,
		0xde7d,
		0xdd7e,
		0xdc80,
		0xdb81,
		0xda82,
		0xda84,
		0xd985,
		0xd886,
		0xd788,
		0xd689,
		0xd58a,
		0xd58c,
		0xd48d,
		0xd38e,
		0xd290,
		0xd191,
		0xd092,
		0xcf93,
		0xce95,
		0xcd96,
		0xcc97,
		0xcc98,
		0xcb9a,
		0xca9b,
		0xc99c,
		0xc89d,
		0xc79f,
		0xc6a0,
		0xc5a1,
		0xc4a2,
		0xc3a4,
		0xc2a5,
		0xc1a6,
		0xc0a7,
		0xbfa8,
		0xbea9,
		0xbdab,
		0xbcac,
		0xbaad,
		0xb9ae,
		0xb8af,
		0xb7b0,
		0xb6b1,
		0xb5b3,
		0xb4b4,
		0xb3b5,
		0xb2b6,
		0xb1b7,
		0xafb8,
		0xaeb9,
		0xadba,
		0xacbb,
		0xabbc,
		0xaabd,
		0xa9be,
		0xa7bf,
		0xa6c0,
		0xa5c1,
		0xa4c3,
		0xa3c4,
		0xa1c5,
		0xa0c5,
		0x9fc6,
		0x9ec7,
		0x9dc8,
		0x9bc9,
		0x9aca,
		0x99cb,
		0x98cc,
		0x96cd,
		0x95ce,
		0x94cf,
		0x92d0,
		0x91d1,
		0x90d2,
		0x8fd3,
		0x8dd3,
		0x8cd4,
		0x8bd5,
		0x89d6,
		0x88d7,
		0x87d8,
		0x85d9,
		0x84d9,
		0x83da,
		0x81db,
		0x80dc,
		0x7fdd,
		0x7ddd,
		0x7cde,
		0x7bdf,
		0x79e0,
		0x78e0,
		0x76e1,
		0x75e2,
		0x74e3,
		0x72e3,
		0x71e4,
		0x6fe5,
		0x6ee5,
		0x6de6,
		0x6be7,
		0x6ae7,
		0x68e8,
		0x67e9,
		0x65e9,
		0x64ea,
		0x63ea,
		0x61eb,
		0x60ec,
		0x5eec,
		0x5ded,
		0x5bed,
		0x5aee,
		0x58ee,
		0x57ef,
		0x55f0,
		0x54f0,
		0x53f1,
		0x51f1,
		0x50f2,
		0x4ef2,
		0x4df3,
		0x4bf3,
		0x4af3,
		0x48f4,
		0x47f4,
		0x45f5,
		0x44f5,
		0x42f6,
		0x41f6,
		0x3ff6,
		0x3ef7,
		0x3cf7,
		0x3af8,
		0x39f8,
		0x37f8,
		0x36f9,
		0x34f9,
		0x33f9,
		0x31fa,
		0x30fa,
		0x2efa,
		0x2dfa,
		0x2bfb,
		0x2afb,
		0x28fb,
		0x27fb,
		0x25fc,
		0x23fc,
		0x22fc,
		0x20fc,
		0x1ffd,
		0x1dfd,
		0x1cfd,
		0x1afd,
		0x19fd,
		0x17fd,
		0x15fe,
		0x14fe,
		0x12fe,
		0x11fe,
		0xffe,
		0xefe,
		0xcfe,
		0xafe,
		0x9fe,
		0x7fe,
		0x6fe,
		0x4fe,
		0x3fe,
		0x1fe,
};

#endif /* MICROSTEP_VALUES_H_ */
