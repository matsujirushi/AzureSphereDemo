#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <alloca.h>

#include "applibs_versions.h"
#include <applibs/log.h>

#include "mt3620_rdb.h"

#include "easyio.h"

#include "Grove.h"
#include "Sensors/GroveTempHumiSHT31.h"
#include "Sensors/GroveRotaryAngleSensor.h"
#include "Sensors/GroveRelay.h"

#include <azureiot/iothubtransportmqtt.h>
#include <azureiot/iothub_device_client_ll.h>
#include "parson.h"

static bool Telemetry = false;
static void* TempHumi;
static void* Rotary;
static void* Buzzer;
static void* Led1Green;
static void* Led2Red;
static void* Led3Red;
static void* Led4Red;

#define INTERVAL			(5)	// [sec.]

#define CONNECTION_STRING   ""

static const char azureIoTCertificatesX[] =
/* Baltimore */
"-----BEGIN CERTIFICATE-----\r\n"
"MIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\r\n"
"RTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\r\n"
"VQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\r\n"
"DTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\r\n"
"ZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\r\n"
"VHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\r\n"
"mD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\r\n"
"IZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\r\n"
"mpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\r\n"
"XmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\r\n"
"dc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\r\n"
"jl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\r\n"
"BE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\r\n"
"DQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\r\n"
"9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\r\n"
"jkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\r\n"
"Epn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\r\n"
"ksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\r\n"
"R9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\r\n"
"-----END CERTIFICATE-----\r\n";

static void SendEventCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
{
	if (result == IOTHUB_CLIENT_CONFIRMATION_OK)
	{
		Log_Debug("%*c(D2C message sent.)\n", 40, ' ');
	}
}

void DeviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char* payload, size_t size, void* userContextCallback)
{
	IOTHUB_DEVICE_CLIENT_LL_HANDLE* clientHandle = (IOTHUB_DEVICE_CLIENT_LL_HANDLE*)userContextCallback;

	char* payloadStr = alloca(size + 1);
	memcpy(payloadStr, payload, size);
	payloadStr[size] = '\0';

	JSON_Value* jsonValue = json_parse_string(payloadStr);
	JSON_Object* jsonObject = json_value_get_object(jsonValue);
	if (update_state == DEVICE_TWIN_UPDATE_COMPLETE)
	{
		jsonObject = json_object_get_object(jsonObject, "desired");
	}
	if (json_object_has_value(jsonObject, "telemetry"))
	{
		Telemetry = json_object_dotget_boolean(jsonObject, "telemetry.value") ? true : false;
		Log_Debug("### Telemetry %s ###\n", Telemetry ? "ON" : "OFF");

		JSON_Value* json2Value = json_value_init_object();
		JSON_Object* json2Object = json_value_get_object(json2Value);
		json_object_set_boolean(json2Object, "telemetry", Telemetry);
		char* json2String = json_serialize_to_string(json2Value);
		IOTHUB_CLIENT_RESULT result = IoTHubDeviceClient_LL_SendReportedState(*clientHandle, json2String, strlen(json2String), NULL, NULL);
		assert(result == IOTHUB_CLIENT_OK);
		json_value_free(json2Value);
	}
	json_value_free(jsonValue);
}

static int DirectMethodCallback(const char* methodName, const unsigned char* payload, size_t size, unsigned char** response, size_t* responseSize, void* userContextCallback)
{
	char* payloadStr = alloca(size + 1);
	memcpy(payloadStr, payload, size);
	payloadStr[size] = '\0';

	//JSON_Value* jsonValue = json_parse_string(payloadStr);
	//JSON_Object* jsonObject = json_value_get_object(jsonValue);
	//int val = json_object_get_boolean(jsonObject, "onoff");
	//json_value_free(jsonValue);

	if (strcmp(methodName, "setBuzzer") == 0)
	{
		Log_Debug("INFO: Trying to invoke method %s\n", methodName);
		//if (val)
		//{
		//	Log_Debug("### setBuzzer(TRUE) ###\n");
		//	DigitalOut_write(Buzzer, 1);
		//	DigitalOut_write(Led2Red, 0);
		//	DigitalOut_write(Led3Red, 0);
		//	DigitalOut_write(Led4Red, 0);
		//}
		//else
		//{
		//	Log_Debug("### setBuzzer(FALSE) ###\n");
		//	DigitalOut_write(Buzzer, 0);
		//	DigitalOut_write(Led2Red, 1);
		//	DigitalOut_write(Led3Red, 1);
		//	DigitalOut_write(Led4Red, 1);
		//}
		for (int i = 0; i < 3; i++)
		{
			DigitalOut_write(Buzzer, 1);
			DigitalOut_write(Led2Red, 0);
			DigitalOut_write(Led3Red, 0);
			DigitalOut_write(Led4Red, 0);
			wait_ms(300);
			DigitalOut_write(Buzzer, 0);
			DigitalOut_write(Led2Red, 1);
			DigitalOut_write(Led3Red, 1);
			DigitalOut_write(Led4Red, 1);
			wait_ms(100);
		}
	}

	const char* resp = "{}";
	*responseSize = strlen(resp);
	*response = malloc(*responseSize);
	memcpy(*response, resp, *responseSize);

	return 0;
}

int main(int argc, char *argv[])
{
    Log_Debug("Application starting.\n");

	// Groveシールド初期化
	int i2cFd;
	GroveShield_Initialize(&i2cFd, 115200);

	// Groveモジュール初期化
	TempHumi = GroveTempHumiSHT31_Open(i2cFd);
	Rotary = GroveRotaryAngleSensor_Init(i2cFd, 0);
	Buzzer = DigitalOut_new(4, 0);
	Led1Green = DigitalOut_new(MT3620_RDB_LED1_GREEN, 1);
	Led2Red = DigitalOut_new(MT3620_RDB_LED2_RED, 1);
	Led3Red = DigitalOut_new(MT3620_RDB_LED3_RED, 1);
	Led4Red = DigitalOut_new(MT3620_RDB_LED4_RED, 1);

	// DeviceClient初期化
#ifdef CONNECTION_STRING
	IOTHUB_DEVICE_CLIENT_LL_HANDLE clientHandle = IoTHubDeviceClient_LL_CreateFromConnectionString(CONNECTION_STRING, MQTT_Protocol);
	assert(clientHandle != NULL);
	IOTHUB_CLIENT_RESULT result;
	result = IoTHubDeviceClient_LL_SetOption(clientHandle, "TrustedCerts", azureIoTCertificatesX);
	assert(result == IOTHUB_CLIENT_OK);
	result = IoTHubDeviceClient_LL_SetDeviceTwinCallback(clientHandle, DeviceTwinCallback, &clientHandle);
	assert(result == IOTHUB_CLIENT_OK);
	result = IoTHubDeviceClient_LL_SetDeviceMethodCallback(clientHandle, DirectMethodCallback, &clientHandle);
	assert(result == IOTHUB_CLIENT_OK);
#endif

	// 永久ループ
	time_t nextTime = time(NULL);
	for (;;)
	{
		// 温湿度センサー
		GroveTempHumiSHT31_Read(TempHumi);
		float temp = GroveTempHumiSHT31_GetTemperature(TempHumi);
		float humi = GroveTempHumiSHT31_GetHumidity(TempHumi);

		// ツマミ
		float angle = (1.0f - GroveRotaryAngleSensor_Read(Rotary)) * 100.0f;

		// 測定値を表示
		Log_Debug("Temp:%.1f[C]  Humi:%.1f[%%]  Angle:%.0f[%%]\n", temp, humi, angle);

		// メッセージの送信
#ifdef CONNECTION_STRING
		if (Telemetry)
		{
			JSON_Value* jsonValue = json_value_init_object();
			JSON_Object* jsonObject = json_value_get_object(jsonValue);
			json_object_set_number(jsonObject, "temperature", temp);
			json_object_set_number(jsonObject, "humidity", humi);
			json_object_set_number(jsonObject, "angle", angle);
			char* jsonString = json_serialize_to_string_pretty(jsonValue);
			IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromString(jsonString);
			assert(messageHandle != NULL);
			json_free_serialized_string(jsonString);
			json_value_free(jsonValue);

			result = IoTHubDeviceClient_LL_SendEventAsync(clientHandle, messageHandle, SendEventCallback, 0);
			assert(result == IOTHUB_CLIENT_OK);
			IoTHubMessage_Destroy(messageHandle);
		}
#endif

		// 待ち
		time_t nowTime = time(NULL);
		nextTime = nextTime + ((nowTime - nextTime) / INTERVAL + 1) * INTERVAL;
		while (time(NULL) < nextTime)
		{
			DigitalOut_write(Led1Green, 0);
#ifdef CONNECTION_STRING
			IoTHubDeviceClient_LL_DoWork(clientHandle);
#endif
			wait_ms(200);

			DigitalOut_write(Led1Green, 1);
#ifdef CONNECTION_STRING
			IoTHubDeviceClient_LL_DoWork(clientHandle);
#endif
			wait_ms(200);
		}
	}

    Log_Debug("Application exiting.\n");
    return 0;
}
