//-----------------------------------------------------------------------------
// Copyright 2015 Thiago Alves
//
// Based on the LDmicro software by Jonathan Westhues
// This file is part of the OpenPLC Software Stack.
//
// OpenPLC is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// OpenPLC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with OpenPLC.  If not, see <http://www.gnu.org/licenses/>.
//------
//
// This file is the hardware layer for the OpenPLC. If you change the platform
// where it is running, you may only need to change this file. All the I/O
// related stuff is here. Basically it provides functions to read and write
// to the OpenPLC internal buffers in order to update I/O state.
// Thiago Alves, Dec 2015
//
// Modified for EtherCAT use by Rainer Kordmaa, Aug 2017
// more work needed here, especially updateEtherCAT() and parseConfig()
// linking from PLC io to EtherCAT io should be defined in config file
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <string.h>
#include <inttypes.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "ladder.h"

using namespace std;
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#include <ethercat.h>
#define EC_TIMEOUTMON 500


extern "C"
{
	OSAL_THREAD_FUNC ecatcheck(void *ptr);
	void updateEtherCAT();
	void startEtherCAT(char *ifname);
	void initEtherCAT();
	void getData(char *line, char *buf, char separator1, char separator2);
	bool parseConfig();
	struct varLink;

	struct varlink{
		int size;
		int plclocation1;
		int plclocation2;
		int ecatslave;
		int ecatbyte;
		int ecatbit;
	};
	
	boolean configured;
	char IOmap[4096];
	OSAL_THREAD_HANDLE thread1;
	int expectedWKC;
	boolean needlf;
	volatile int wkc;
	boolean inOP;
	uint8 currentgroup = 0;
	int num_devices = 0;		//expected count of devices
	char ifbuf[100];			//ifname
	varlink * Ilinks;
	varlink * Olinks;
	int Ilinkcount;
	int Olinkcount;
	
	
	void updateEtherCAT()
	{
		if(!configured)return;
		ec_send_processdata();
		wkc = ec_receive_processdata(EC_TIMEOUTRET);
		
		for(int i = 0; i < Ilinkcount; i++){
			if(Ilinks[i].size == 1){
				//Boolean handling
				if (bool_input[Ilinks[i].plclocation1 / 8][Ilinks[i].plclocation2 % 8] != NULL) {
					*bool_input[Ilinks[i].plclocation1 / 8][Ilinks[i].plclocation2 % 8] = bitRead(*(ec_slave[Ilinks[i].ecatslave].inputs + Ilinks[i].ecatbyte), Ilinks[i].ecatbit);
				}
			}
			else if(Ilinks[i].size == 8){
				//SINT handling
				if (sint_input[Ilinks[i].plclocation1] != NULL){
					*sint_input[Ilinks[i].plclocation1] = *(ec_slave[Ilinks[i].ecatslave].inputs + Ilinks[i].ecatbyte);
				}
			}
			else if(Ilinks[i].size == 16){
				//INT handling
				if (int_input[Ilinks[i].plclocation1] != NULL){
					memcpy(int_input[Ilinks[i].plclocation1], ec_slave[Ilinks[i].ecatslave].inputs + Ilinks[i].ecatbyte, 2);
				}
			}
			else if(Ilinks[i].size == 32){
				//DINT handling
				if (dint_input[Ilinks[i].plclocation1] != NULL){
					memcpy(dint_input[Ilinks[i].plclocation1], ec_slave[Ilinks[i].ecatslave].inputs + Ilinks[i].ecatbyte, 4);
				}
			}
		}

		for(int i = 0; i < Olinkcount; i++){
			if(Olinks[i].size == 1){
				//Boolean handling
				if (bool_output[Olinks[i].plclocation1 / 8][Olinks[i].plclocation2 % 8] != NULL) {
					bitWrite(*(ec_slave[Olinks[i].ecatslave].outputs + Olinks[i].ecatbyte), Olinks[i].ecatbit, *bool_output[Olinks[i].plclocation1 / 8][Olinks[i].plclocation2 % 8]);
				}
			}
			else if(Olinks[i].size == 8){
				//SINT handling
				if (sint_output[Olinks[i].plclocation1] != NULL) {
					*(ec_slave[Olinks[i].ecatslave].outputs + Olinks[i].ecatbyte) = *sint_output[Olinks[i].plclocation1];
				}
			}
			else if(Olinks[i].size == 16){
				//INT handling
				if (int_output[Olinks[i].plclocation1] != NULL) {
					memcpy(ec_slave[Olinks[i].ecatslave].outputs + Olinks[i].ecatbyte, int_output[Olinks[i].plclocation1], 2);
				}
			}
			else if(Olinks[i].size == 32){
				//DINT handling
				if (dint_output[Olinks[i].plclocation1] != NULL) {
					memcpy(ec_slave[Olinks[i].ecatslave].outputs + Olinks[i].ecatbyte, dint_output[Olinks[i].plclocation1], 4);
				}
			}
		}

		needlf = TRUE;
	}

	void startEtherCAT(char *ifname)
	{
		int i, j, chk;
		needlf = FALSE;
		inOP = FALSE;

		printf("Starting simple test\n");

		/* initialise SOEM, bind socket to ifname */
		if (ec_init(ifname))
		{
			printf("ec_init on %s succeeded.\n", ifname);
			/* find and auto-config slaves */


			if (ec_config_init(FALSE) > 0)
			{
				for (i = 1; i <= ec_slavecount; i++) {
					printf("%s found at position %d\n", ec_slave[i].name, i);
				}
				printf("%d slaves found and configured.\n", ec_slavecount);

				ec_config_map(&IOmap);

				ec_configdc();

				printf("Slaves mapped, state to SAFE_OP.\n");
				/* wait for all slaves to reach SAFE_OP state */
				ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);


				printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

				printf("Request operational state for all slaves\n");
				expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
				printf("Calculated workcounter %d\n", expectedWKC);
				ec_slave[0].state = EC_STATE_OPERATIONAL;
				/* send one valid process data to make outputs in slaves happy*/
				ec_send_processdata();
				ec_receive_processdata(EC_TIMEOUTRET);
				/* request OP state for all slaves */
				ec_writestate(0);
				chk = 40;
				/* wait for all slaves to reach OP state */
				do
				{
					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
				} while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
				if (ec_slave[0].state == EC_STATE_OPERATIONAL)
				{
					printf("Operational state reached for all slaves.\n");

					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					inOP = TRUE;
					needlf = TRUE;
				}
				else
				{
					printf("Not all slaves reached operational state.\n");
					ec_readstate();
					for (i = 1; i <= ec_slavecount; i++)
					{
						if (ec_slave[i].state != EC_STATE_OPERATIONAL)
						{
							printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
								i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
						}
					}
				}
			}
			else
			{
				printf("No slaves found!\n");
			}
		}
		else
		{
			printf("No socket connection on %s\nExcecute as root\n", ifname);
		}
	}

	OSAL_THREAD_FUNC ecatcheck(void *ptr)
	{
		int slave;

		while (1)
		{
			if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
			{
				if (needlf)
				{
					needlf = FALSE;
					printf("\n");
				}
				/* one ore more slaves are not responding */
				ec_group[currentgroup].docheckstate = FALSE;
				ec_readstate();
				for (slave = 1; slave <= ec_slavecount; slave++)
				{
					if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
					{
						ec_group[currentgroup].docheckstate = TRUE;
						if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
						{
							printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
							ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
							ec_writestate(slave);
						}
						else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
						{
							printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
							ec_slave[slave].state = EC_STATE_OPERATIONAL;
							ec_writestate(slave);
						}
						else if (ec_slave[slave].state > EC_STATE_NONE)
						{
							if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
							{
								ec_slave[slave].islost = FALSE;
								printf("MESSAGE : slave %d reconfigured\n", slave);
							}
						}
						else if (!ec_slave[slave].islost)
						{
							/* re-check state */
							ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
							if (ec_slave[slave].state == EC_STATE_NONE)
							{
								ec_slave[slave].islost = TRUE;
								printf("ERROR : slave %d lost\n", slave);
							}
						}
					}
					if (ec_slave[slave].islost)
					{
						if (ec_slave[slave].state == EC_STATE_NONE)
						{
							if (ec_recover_slave(slave, EC_TIMEOUTMON))
							{
								ec_slave[slave].islost = FALSE;
								printf("MESSAGE : slave %d recovered\n", slave);
							}
						}
						else
						{
							ec_slave[slave].islost = FALSE;
							printf("MESSAGE : slave %d found\n", slave);
						}
					}
				}
				if (!ec_group[currentgroup].docheckstate)
					printf("OK : all slaves resumed OPERATIONAL.\n");
			}
			osal_usleep(10000);
		}
	}

}

void initEtherCAT()
{
	osal_thread_create(&thread1, 128000, (void*) &ecatcheck, (void*) &ctime);
	startEtherCAT(ifbuf);
}

//-----------------------------------------------------------------------------
// Finds the data between the separators on the line provided
//-----------------------------------------------------------------------------
void getData(char *line, char *buf, char separator1, char separator2)
{
	int i=0, j=0;
	buf[j] = '\0';

	while (line[i] != separator1 && line[i] != '\0')
	{
		i++;
	}
	i++;

	while (line[i] != separator2 && line[i] != '\0')
	{
		buf[j] = line[i];
		i++;
		j++;
		buf[j] = '\0';
	}
}
bool parseConfig()
{
	string line;
	char line_str[1024];
	int OlinkPos = 0;
	int IlinkPos = 0;
	ifstream cfgfile("mbconfig.cfg");

	if (cfgfile.is_open())
	{
		while (getline(cfgfile, line))
		{
			strncpy(line_str, line.c_str(), 1024);
			if (line_str[0] != '#' && strlen(line_str) > 1)
			{
				if (!strncmp(line_str, "Num_Devices", 11))
				{
					char temp_buffer[5];
					getData(line_str, temp_buffer, '"', '"');
					num_devices = atoi(temp_buffer);
				}
				if (!strncmp(line_str, "ifname", 6))
				{
					char temp_buffer[100];
					getData(line_str, temp_buffer, '"', '"');
					strcpy(ifbuf, temp_buffer);
				}
				if (!strncmp(line_str, "Icount", 6))
				{
					char temp_buffer[100];
					getData(line_str, temp_buffer, '"', '"');
					Ilinkcount = atoi(temp_buffer);
					Ilinks = (varlink*) malloc(sizeof(varlink) * Ilinkcount);
				}
				if (!strncmp(line_str, "Ocount", 6))
				{
					char temp_buffer[100];
					getData(line_str, temp_buffer, '"', '"');
					Olinkcount = atoi(temp_buffer);
					Olinks = (varlink*) malloc(sizeof(varlink) * Olinkcount);
				}
				if (!strncmp(line_str, "Ilink", 5))
				{
					char parameter[10];
					char temp_buffer[100];
					varlink tempLink;
					getData(line_str, temp_buffer, '"', '"');
					
					stringstream ss;
					ss << temp_buffer;
					string toks[6];
					ss >> toks[0] >> toks[1] >> toks[2] >> toks[3] >> toks[4] >> toks[5];

					tempLink.size = atoi(toks[0].c_str());
					tempLink.plclocation1 = atoi(toks[1].c_str());
					tempLink.plclocation2 = atoi(toks[2].c_str());
					tempLink.ecatslave = atoi(toks[3].c_str());
					tempLink.ecatbyte = atoi(toks[4].c_str());
					tempLink.ecatbit = atoi(toks[5].c_str());
					Ilinks[IlinkPos] = tempLink;
					printf("Ilink %d registered size=%d, plcInput=I%s%d.%d, slave=%d, byte=%d.%d\n", 
							IlinkPos + 1,
							Ilinks[IlinkPos].size, 
							Ilinks[IlinkPos].size == 1 ? "X" : Ilinks[IlinkPos].size == 8 ? "B" : Ilinks[IlinkPos].size == 16 ? "W" : Ilinks[IlinkPos].size == 32 ? "D" : "NotSupported",
							Ilinks[IlinkPos].plclocation1, 
							Ilinks[IlinkPos].plclocation2, 
							Ilinks[IlinkPos].ecatslave, 
							Ilinks[IlinkPos].ecatbyte, 
							Ilinks[IlinkPos].ecatbit);
					IlinkPos++;
				}
				if (!strncmp(line_str, "Olink", 5))
				{
					char parameter[10];
					char temp_buffer[100];
					varlink tempLink;
					getData(line_str, temp_buffer, '"', '"');
					
					stringstream ss;
					ss << temp_buffer;
					string toks[6];
					ss >> toks[0] >> toks[1] >> toks[2] >> toks[3] >> toks[4] >> toks[5];

					tempLink.size = atoi(toks[0].c_str());
					tempLink.plclocation1 = atoi(toks[1].c_str());
					tempLink.plclocation2 = atoi(toks[2].c_str());
					tempLink.ecatslave = atoi(toks[3].c_str());
					tempLink.ecatbyte = atoi(toks[4].c_str());
					tempLink.ecatbit = atoi(toks[5].c_str());
					Olinks[OlinkPos] = tempLink;
					printf("Olink %d registered size=%d, plcInput=I%s%d.%d, slave=%d, byte=%d.%d\n",
							OlinkPos + 1,
							Olinks[OlinkPos].size, 
							Olinks[OlinkPos].size == 1 ? "X" : Olinks[OlinkPos].size == 8 ? "B" : Olinks[OlinkPos].size == 16 ? "W" : Olinks[OlinkPos].size == 32 ? "D" : "NotSupported",
							Olinks[OlinkPos].plclocation1, 
							Olinks[OlinkPos].plclocation2, 
							Olinks[OlinkPos].ecatslave, 
							Olinks[OlinkPos].ecatbyte, 
							Olinks[OlinkPos].ecatbit);
					OlinkPos++;
				}
				
			}
		}
		printf("ifname = %s\n", ifbuf);
		printf("Num_Devices = %d\n", num_devices);
		printf("I links = %d\n", Ilinkcount);
		printf("O links = %d\n", Olinkcount);
		return true;
	}
	else { 
		printf("failed to open config file\n");
		return false; 
	}
}
//-----------------------------------------------------------------------------
// This function is called by the OpenPLC in a loop. Here the internal buffers
// must be updated to reflect the actual I/O state. The mutex bufferLock
// must be used to protect access to the buffers on a threaded environment.
//-----------------------------------------------------------------------------
void updateBuffers()
{
	pthread_mutex_lock(&bufferLock); //lock mutex

	updateEtherCAT();

	pthread_mutex_unlock(&bufferLock); //unlock mutex
}
//-----------------------------------------------------------------------------
// This function is called by the main OpenPLC routine when it is initializing.
// Hardware initialization procedures should be here.
//-----------------------------------------------------------------------------
void initializeHardware()
{
	configured = parseConfig();
	if (configured) {
		initEtherCAT();
	}
	else {
		printf("\"mbconfig.cfg\" missing, required to start EtherCAT\n");
	}
}







