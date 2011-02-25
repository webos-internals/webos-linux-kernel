/*
 * charger_detector.h
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

extern int charger_schedule_detection(void);
extern int charger_cancel_detection(void);
extern int charger_vbus_lost(void);

/* vbus_watcher depends on these */
extern int transceiver_vbus_presence(int *vbus);
extern int transceiver_is_pullup_attached(int *pullup);
extern int transceiver_single_ended_state(int *dplus, int *dminus);
