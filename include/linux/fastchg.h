/*
 * Author: Chad Froebel <chadfroebel@gmail.com>
 *
 * Port to Osprey : engstk <eng.stk@sapo.pt>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _LINUX_FASTCHG_H
#define _LINUX_FASTCHG_H

extern int force_fast_charge;
extern int fast_charge_level;
extern int usb_fast_charge_level;


#define IVBUS_FAST_CHARGE_LEVELS	"For AC: Anything between range of 500 - 1900 mA (Caution: Exceed 1900mA at your own risk)"
#define USB_FAST_CHARGE_LEVELS	"For USB: Anything between range of 500 - 1500 mA"

#endif
