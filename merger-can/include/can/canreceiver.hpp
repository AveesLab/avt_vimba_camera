/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * 03_CanReceiver.h - PCANBasic Example: CanReceiver
 *
 * Copyright (C) 2001-2020  PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:    <linux@peak-system.com>
 * Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
 * 	    	    Romain Tissier <r.tissier@peak-system.com>
 */
#include "linux_interop.h"
#include "PCANBasic.h"
#include "monitor/types.h"
#include <vector>

class CanReceiver
{
private:
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	const bool IsFD = false;
	const TPCANBaudrate Bitrate = PCAN_BAUD_500K;
	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");

public:
	CanReceiver();
	~CanReceiver();

	int GetMessage(ObjectDetection& detection);
	TPCANStatus SendMessage(std::vector<int>& base_timestamp_);

private:
	void ReadMessages();
	TPCANStatus ReadMessageFD();
	TPCANStatus ReadMessage();
	void ShowCurrentConfiguration();
	void ShowStatus(TPCANStatus status);
	void FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD);
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);
	void GetFormattedError(TPCANStatus error, LPSTR buffer);
	void ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer);
	std::string GetMsgTypeString(TPCANMessageType msgType);
	int GetLengthFromDLC(BYTE dlc);
	std::string GetDataString(BYTE data[], TPCANMessageType msgType, int dataLength);
};
