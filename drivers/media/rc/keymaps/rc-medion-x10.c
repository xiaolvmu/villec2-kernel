/*
 * Medion X10 RF remote keytable
 *
 * Copyright (C) 2011 Anssi Hannula <anssi.hannula@?ki.fi>
 *
 * This file is based on a keytable provided by
 * Jan Losinski <losinski@wh2.tu-dresden.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <media/rc-map.h>

static struct rc_map_table medion_x10[] = {
	{ 0x2c, KEY_TV },    
	{ 0x2d, KEY_VCR },   
	{ 0x04, KEY_DVD },   
	{ 0x06, KEY_AUDIO }, 

	{ 0x2e, KEY_RADIO },     
	{ 0x05, KEY_DIRECTORY }, 
	{ 0x2f, KEY_INFO },      
	{ 0x30, KEY_LIST },      

	{ 0x1b, KEY_SETUP }, 
	{ 0x31, KEY_VIDEO }, 

	{ 0x08, KEY_VOLUMEDOWN },  
	{ 0x09, KEY_VOLUMEUP },    
	{ 0x0b, KEY_CHANNELUP },   
	{ 0x0c, KEY_CHANNELDOWN }, 
	{ 0x00, KEY_MUTE },        

	{ 0x32, KEY_RED }, 
	{ 0x33, KEY_GREEN }, 
	{ 0x34, KEY_YELLOW }, 
	{ 0x35, KEY_BLUE }, 
	{ 0x16, KEY_TEXT }, 

	{ 0x0d, KEY_1 },
	{ 0x0e, KEY_2 },
	{ 0x0f, KEY_3 },
	{ 0x10, KEY_4 },
	{ 0x11, KEY_5 },
	{ 0x12, KEY_6 },
	{ 0x13, KEY_7 },
	{ 0x14, KEY_8 },
	{ 0x15, KEY_9 },
	{ 0x17, KEY_0 },
	{ 0x1c, KEY_SEARCH }, 
	{ 0x20, KEY_DELETE }, 

	{ 0x36, KEY_KEYBOARD }, 
	{ 0x18, KEY_SCREEN },   

	{ 0x1a, KEY_UP },    
	{ 0x22, KEY_DOWN },  
	{ 0x1d, KEY_LEFT },  
	{ 0x1f, KEY_RIGHT }, 
	{ 0x1e, KEY_OK },    

	{ 0x37, KEY_SELECT }, 
	{ 0x38, KEY_EDIT },   

	{ 0x24, KEY_REWIND },   
	{ 0x25, KEY_PLAY },     
	{ 0x26, KEY_FORWARD },  
	{ 0x27, KEY_RECORD },   
	{ 0x28, KEY_STOP },     
	{ 0x29, KEY_PAUSE },    

	{ 0x21, KEY_PREVIOUS },        
	{ 0x39, KEY_SWITCHVIDEOMODE }, 
	{ 0x23, KEY_NEXT },            
	{ 0x19, KEY_MENU },            
	{ 0x3a, KEY_LANGUAGE },        

	{ 0x02, KEY_POWER }, 
};

static struct rc_map_list medion_x10_map = {
	.map = {
		.scan    = medion_x10,
		.size    = ARRAY_SIZE(medion_x10),
		.rc_type = RC_TYPE_OTHER,
		.name    = RC_MAP_MEDION_X10,
	}
};

static int __init init_rc_map_medion_x10(void)
{
	return rc_map_register(&medion_x10_map);
}

static void __exit exit_rc_map_medion_x10(void)
{
	rc_map_unregister(&medion_x10_map);
}

module_init(init_rc_map_medion_x10)
module_exit(exit_rc_map_medion_x10)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anssi Hannula <anssi.hannula@iki.fi>");
