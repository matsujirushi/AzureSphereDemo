#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <unistd.h>

#define UART_STRUCTS_VERSION 1

#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/uart.h>

static int UartFd;
static void (*GpsMessageReceivedCallback)(const char* message) = NULL;

static void GpsInit(void)
{
	UART_Config uartConfig;
	UART_InitConfig(&uartConfig);
	uartConfig.baudRate = 9600;
	uartConfig.flowControl = UART_FlowControl_None;
	UartFd = UART_Open(7, &uartConfig);	// MT3620_ISU3_UART
	assert(UartFd >= 0);
}

static void GpsAttachMessageReceived(void (*callback)(const char* message))
{
	GpsMessageReceivedCallback = callback;
}

static void GpsDoWork(void)
{
	static char receiveBuffer[256];
	static int receivedSize = 0;

	assert((int)sizeof(receiveBuffer) - receivedSize >= 1);
	ssize_t partialReceivedSize = read(UartFd, &receiveBuffer[receivedSize], (size_t)((int)sizeof(receiveBuffer) - receivedSize));
	assert(partialReceivedSize >= 0);
	if (partialReceivedSize >= 1) {
		receivedSize += partialReceivedSize;
		receiveBuffer[receivedSize] = '\0';

		char* begin = receiveBuffer;
		char* end;
		while ((end = strchr(begin, '\r')) != NULL) {
			*end = '\0';

			if (GpsMessageReceivedCallback != NULL) GpsMessageReceivedCallback(begin);

			begin = end + 1;
			if (*begin == '\n') begin++;
		}
		if (begin != receiveBuffer) {
			receivedSize -= begin - receiveBuffer;
			memmove(receiveBuffer, begin, (size_t)receivedSize);
		}
	}
}

void GpsMessageReceived(const char* message)
{
	if (strncmp(message, "$GPGGA,", 7) == 0) {	// Global Positioning System Fix Data
		char* utcTime = strchr(message, ',');
		assert(utcTime != NULL);
		utcTime++;

		char* latitude = strchr(utcTime, ',');
		assert(latitude != NULL);
		latitude++;

		char* latitudeNS = strchr(latitude, ',');
		assert(latitudeNS != NULL);
		latitudeNS++;

		char* longitude = strchr(latitudeNS, ',');
		assert(longitude != NULL);
		longitude++;

		char* longitudeNS = strchr(longitude, ',');
		assert(longitudeNS != NULL);
		longitudeNS++;

		Log_Debug("lat:%lf%c long:%lf%c\n", atof(latitude), *latitudeNS,atof(longitude), *longitudeNS);
	}
}

int main(void)
{
	Log_Debug("GPS application starting.\n");

	GpsInit();
	GpsAttachMessageReceived(GpsMessageReceived);

	const struct timespec sleepTime = { 0, 10e6 };
    while (true) {
		GpsDoWork();
		nanosleep(&sleepTime, NULL);
	}
}