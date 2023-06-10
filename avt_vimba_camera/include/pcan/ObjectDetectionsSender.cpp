#include "ObjectDetectionsSender.hpp"

ObjectDetectionsSender::ObjectDetectionsSender(unsigned int can_id, int time_interval) : can_id_(can_id), time_interval_(time_interval)
{
	ShowCurrentConfiguration(); // Shows the current parameters configuration

	TPCANStatus stsResult;
	// Initialization of the selected channel
	if (IsFD)
		stsResult = CAN_InitializeFD(PcanHandle, BitrateFD);
	else
		stsResult = CAN_Initialize(PcanHandle, Bitrate);

	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout << "Can not initialize. Please check the defines in the code.\n";
		ShowStatus(stsResult);
		std::cout << "\n";
		std::cout << "Closing...\n";
		return;
	}

	// Writing messages...
	std::cout << "Successfully initialized.\n";
}

ObjectDetectionsSender::~ObjectDetectionsSender()
{
	CAN_Uninitialize(PCAN_NONEBUS);
}

void ObjectDetectionsSender::WriteMessages(double time_stamp, std::vector<ObjectDetection>& detections)
{
	TPCANStatus stsResult;

	if (IsFD)
		stsResult = WriteMessageFD(time_stamp, detections);
	else
		stsResult = WriteMessage(time_stamp, detections);

	// Checks if the message was sent
	if (stsResult != PCAN_ERROR_OK)
		ShowStatus(stsResult);
	else
		std::cerr << "Message was successfully SENT\n";
}

TPCANStatus ObjectDetectionsSender::WriteMessage(double time_stamp, std::vector<ObjectDetection>& detections)
{
	// Sends a CAN message with extended ID, and 8 data bytes
	TPCANMsg msgCanMessage;
	msgCanMessage.ID = this->can_id_;
	msgCanMessage.LEN = (BYTE)8;
	msgCanMessage.MSGTYPE = PCAN_MESSAGE_EXTENDED;

	int remaked_time_stamp = static_cast<int>(time_stamp * 1000.0) % 60000;

	for (size_t index = 0; index < detections.size(); index++)
	{
		msgCanMessage.DATA[0] = remaked_time_stamp >> 8;
		msgCanMessage.DATA[1] = remaked_time_stamp;
		msgCanMessage.DATA[2] = detections[index].id;
		msgCanMessage.DATA[3] = detections[index].center_x  >> 3;
		msgCanMessage.DATA[4] = ((detections[index].center_x  & 7) << 5 ) | (detections[index].center_y >>5);
		msgCanMessage.DATA[5] = ((detections[index].center_y & 31) << 3) |  (detections[index].width_half >> 7);
		msgCanMessage.DATA[6] = ((detections[index].width_half & 127) << 1) | (detections[index].height_half>> 8);
		msgCanMessage.DATA[7] =  detections[index].height_half;

		TPCANStatus stsResult = CAN_Write(PcanHandle, &msgCanMessage);

		if (stsResult != PCAN_ERROR_OK)
		{
			return stsResult;
		}

		usleep(this->time_interval_);
	}

	TPCANMsg msgCanMessage2;
	msgCanMessage2.ID = this->can_id_;
	msgCanMessage2.LEN = (BYTE)2;
	msgCanMessage2.MSGTYPE = PCAN_MESSAGE_EXTENDED;

	msgCanMessage2.DATA[0] = remaked_time_stamp >> 8;
	msgCanMessage2.DATA[1] = remaked_time_stamp;

	TPCANStatus stsResult = CAN_Write(PcanHandle, &msgCanMessage2);

	if (stsResult != PCAN_ERROR_OK)
	{
		return stsResult;
	}
}

TPCANStatus ObjectDetectionsSender::WriteMessageFD(double time_stamp, std::vector<ObjectDetection>& detections)
{
	// Sends a CAN-FD message with standard ID, 64 data bytes, and bitrate switch
	TPCANMsgFD msgCanMessageFD;
	msgCanMessageFD.ID = this->can_id_;
	msgCanMessageFD.DLC = 15;
	msgCanMessageFD.MSGTYPE = PCAN_MESSAGE_FD | PCAN_MESSAGE_BRS;

	for (size_t index = 0; index < detections.size(); index++)
	{
		for (BYTE i = 0; i < 8; i++)
		{
			msgCanMessageFD.DATA[i] = i;
		}

		TPCANStatus stsResult = CAN_WriteFD(PcanHandle, &msgCanMessageFD);

		if (stsResult != PCAN_ERROR_OK)
		{
			return stsResult;
		}

		usleep(this->time_interval_);
	}
}

void ObjectDetectionsSender::ShowCurrentConfiguration()
{
	std::cout << "Parameter values used\n";
	std::cout << "----------------------\n";
	char buffer[MAX_PATH];
	FormatChannelName(PcanHandle, buffer, IsFD);
	std::cout << "* PCANHandle: " << buffer << "\n";
	if (IsFD)
		std::cout << "* IsFD: True\n";
	else
		std::cout << "* IsFD: False\n";
	ConvertBitrateToString(Bitrate, buffer);
	std::cout << "* Bitrate: " << buffer << "\n";
	std::cout << "* BitrateFD: " << BitrateFD << "\n";
	std::cout << "\n";
}

void ObjectDetectionsSender::ShowStatus(TPCANStatus status)
{
	std::cerr << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cerr << buffer << "\n";
	std::cerr << "=========================================================================================\n";
}

void ObjectDetectionsSender::FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD)
{
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	GetTPCANHandleName(handle, handleBuffer);
	if (isFD)
		sprintf_s(buffer, MAX_PATH, "%s:FD %d (%Xh)", handleBuffer, byChannel, handle);
	else
		sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void ObjectDetectionsSender::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
{
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void ObjectDetectionsSender::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void ObjectDetectionsSender::ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer)
{
	switch (bitrate)
	{
	case PCAN_BAUD_1M:
		strcpy_s(buffer, MAX_PATH, "1 MBit/sec");
		break;
	case PCAN_BAUD_800K:
		strcpy_s(buffer, MAX_PATH, "800 kBit/sec");
		break;
	case PCAN_BAUD_500K:
		strcpy_s(buffer, MAX_PATH, "500 kBit/sec");
		break;
	case PCAN_BAUD_250K:
		strcpy_s(buffer, MAX_PATH, "250 kBit/sec");
		break;
	case PCAN_BAUD_125K:
		strcpy_s(buffer, MAX_PATH, "125 kBit/sec");
		break;
	case PCAN_BAUD_100K:
		strcpy_s(buffer, MAX_PATH, "100 kBit/sec");
		break;
	case PCAN_BAUD_95K:
		strcpy_s(buffer, MAX_PATH, "95,238 kBit/sec");
		break;
	case PCAN_BAUD_83K:
		strcpy_s(buffer, MAX_PATH, "83,333 kBit/sec");
		break;
	case PCAN_BAUD_50K:
		strcpy_s(buffer, MAX_PATH, "50 kBit/sec");
		break;
	case PCAN_BAUD_47K:
		strcpy_s(buffer, MAX_PATH, "47,619 kBit/sec");
		break;
	case PCAN_BAUD_33K:
		strcpy_s(buffer, MAX_PATH, "33,333 kBit/sec");
		break;
	case PCAN_BAUD_20K:
		strcpy_s(buffer, MAX_PATH, "20 kBit/sec");
		break;
	case PCAN_BAUD_10K:
		strcpy_s(buffer, MAX_PATH, "10 kBit/sec");
		break;
	case PCAN_BAUD_5K:
		strcpy_s(buffer, MAX_PATH, "5 kBit/sec");
		break;
	default:
		strcpy_s(buffer, MAX_PATH, "Unknown Bitrate");
		break;
	}
}