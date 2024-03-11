#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <cstdio>
#include <cassert>

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <WinSock2.h>
#include <Windows.h> // sleep 사용 위해
#include <stdint.h> // int32_t 사용 위해
#include <thread>

#pragma comment(lib, "ws2_32.lib")

#define PORT 4578

void error_handling(const char *message);

/*******************************************************************************
 TCP/IP Socket Communication
*******************************************************************************/
WSADATA wsaData;
SOCKET serv_sock, clnt_sock;
SOCKADDR_IN serv_adr, clnt_adr;

void accpetclients() {

	int clnt_size = sizeof(clnt_adr);
	clnt_sock = accept(serv_sock, (SOCKADDR*)&clnt_adr, &clnt_size);

	if (clnt_sock == INVALID_SOCKET) {
		error_handling("accept() error");
		closesocket(clnt_sock);
		closesocket(serv_sock);
		WSACleanup();
		return;
	}

	fputs("Connected to client! \n", stderr);

	return;
}

void openSocket() {

	if (WSAStartup(MAKEWORD(2, 2), &wsaData)) {
		error_handling("WSA error");
		return;
	}

	serv_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (serv_sock == INVALID_SOCKET) {
		error_handling("socket error");
		closesocket(serv_sock);
		WSACleanup();
		return;
	}

	serv_adr.sin_family = AF_INET;
	serv_adr.sin_port = htons(PORT);
	serv_adr.sin_addr.s_addr = htonl(INADDR_ANY);


	if (bind(serv_sock, (sockaddr*)&serv_adr, sizeof(serv_adr))) {
		error_handling("bind error");
		closesocket(serv_sock);
		WSACleanup();
		return;
	}

	if (listen(serv_sock, SOMAXCONN)) {
		error_handling("listen error");
		closesocket(serv_sock);
		WSACleanup();
		return;
	}

	fputs("Server is Ready \n", stderr);

	accpetclients();
}

/*******************************************************************************
 Callback Functions for Scheduler
*******************************************************************************/
HDCallbackCode HDCALLBACK PlaneCallback(void* pUserData);

HDSchedulerHandle hPlaneCallback = HD_INVALID_HANDLE;

HDCallbackCode HDCALLBACK PlaneCallback(void *pUserData)
{
	HDErrorInfo error;
	hduVector3Dd position;
	hduVector3Dd velocity;

	const int basePlane = 0; //[mm] plane의 높이 조절
	const double maxVelocity = 100.0; //[mm/s] 최대 허용 속도 (mm/s)
	int directionFlag = 0;
	double velocityError = 0;
	const double kp = .1;

	HHD hHD = hdGetCurrentDevice();
	hdBeginFrame(hHD);

	// Get the position of the device.
	hdGetDoublev(HD_CURRENT_POSITION, position);
	hdGetDoublev(HD_CURRENT_VELOCITY, velocity);

	double penetrationDistance = fabs(position[1] - basePlane);

	// plane motion
	if (position[1] < basePlane) {
		directionFlag = 1;
	}
	else if (position[1] > basePlane) {
		directionFlag = -1;
	}
	else { directionFlag = 0; }

	// limit velocity
	//if ((velocity[2] > maxVelocity) || (velocity[2] < -maxVelocity)) {
	//	velocityError = maxVelocity - velocity[2];	}
	//else { velocityError = 0; }


	hduVector3Dd forceDirection(0, directionFlag, 0);

	// Hooke's law explicitly:
	double k = 0.5;
	hduVector3Dd x = penetrationDistance * forceDirection;
	hduVector3Dd f = k * x;
	//f[2] = kp * velocityError;

	hdSetDoublev(HD_CURRENT_FORCE, f);
	hdEndFrame(hHD);

	/* Check for errors and abort the callback if a scheduler error	is detected. */
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error,
			"Error detected while rendering xy plane\n");

		if (hduIsSchedulerError(&error))
		{
			return HD_CALLBACK_DONE;
		}
	}

	return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Main function
 ******************************************************************************/
int main(int argc, char* argv[])
{
	/****** Haptic Device ******/
	HDErrorInfo error;

	// Initialize the default haptic device.
	HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to initialize haptic device");
		fprintf(stderr, "\nPress any key to quit.\n");
		getch();
		return -1;
	}

	/* Schedule the main callback that will render forces to the device. */
	hPlaneCallback = hdScheduleAsynchronous(PlaneCallback, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY);
	//hVelLockCallback = hdScheduleAsynchronous(VelLockCallback, (void*)0, HD_MIN_SCHEDULER_PRIORITY);

	// Start Scheduler
	hdEnable(HD_FORCE_OUTPUT);
	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError()))
	{
		hduPrintError(stderr, &error, "Failed to start the scheduler");
		fprintf(stderr, "\nPress any key to quit.\n");
		getch();
		return -1;
	}


	/****** TCP/IP ******/
	openSocket();

	char answer;
	std::cout << "Do you want to start? (Y/N): ";
	std::cin >> answer;
	if (answer != 'y' && answer != 'Y') {
		answer = 'Y';
	}


	/****** Loop ******/
	while (1) {

		hduVector3Dd position;
		hduVector3Dd gimbalAngles;
		hduVector3Dd jointAngles;

		/* Get the current position of the device. */
		hdGetDoublev(HD_CURRENT_POSITION, position);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbalAngles);
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);

		printf("displacement: %.4f mm \t gimbal angle: %.4f deg\n", position[2], gimbalAngles[0] * 180 / 3.141592653589);
		//printf("%f\t%f\t%f\n\n", gimbalAngles[0], gimbalAngles[1], gimbalAngles[2]);
		//printf("%f\t%f\t%f\n\n", jointAngles[0], jointAngles[1], jointAngles[2]);

		position = 1000 * position; //um
		gimbalAngles = (1000 * gimbalAngles * 180 / 3.141592653589); //mdeg

		//**TCP/IP**//
		//*****Sending Data to Client*****//
		int32_t data[6] = { position[0], position[2], position[1], gimbalAngles[0], gimbalAngles[1], gimbalAngles[2] };
		send(clnt_sock, (char*)&data, sizeof(data), 0);

		Sleep(1);
	}


	/****** Shutdown ******/
	closesocket(clnt_sock);
	closesocket(serv_sock);
	WSACleanup();

	hdStopScheduler();
	hdUnschedule(hPlaneCallback);
	hdDisableDevice(hHD);

	return 0;
}


/*******************************************************************************
 Error function
 ******************************************************************************/
void error_handling(const char *message)
{
	fputs(message, stderr);
	fputc('\n', stderr);
	exit(1);
}
