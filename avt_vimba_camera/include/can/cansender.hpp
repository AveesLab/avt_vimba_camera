#include "linux_interop.h"
#include "PCANBasic.h"

#include <vector>
#include "objectdetection/types.h"

class CanSender
{
private:
	const TPCANHandle PcanHandle = PCAN_USBBUS1;
	const bool IsFD = false;
	const TPCANBaudrate Bitrate = PCAN_BAUD_500K;
	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1");

	unsigned int can_id_;
	int time_interval_;

public:
	CanSender(int node_index, int time_interval);
	~CanSender();

	void WriteMessages(double time_stamp, std::vector<ObjectDetection>& detections, int limits);
	void ClusterSyncRequest();
	void ReadMessage(int& index, int& time);

private:
	TPCANStatus WriteMessage(double time_stamp, std::vector<ObjectDetection>& detections, int limits);
	void ShowCurrentConfiguration();
	void ShowStatus(TPCANStatus status);
	void FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD);
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);
	void GetFormattedError(TPCANStatus error, LPSTR buffer);
	void ConvertBitrateToString(TPCANBaudrate bitrate, LPSTR buffer);
};
