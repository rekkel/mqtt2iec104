#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include "cs104_slave.h"

#include "hal_thread.h"
#include "hal_time.h"

#include <mqtt.h>
#include <posix_sockets.h>
/***
	MQTT stuf
***/

char *digitals[8];

struct reconnect_state_t {
    const char* hostname;
    const char* port;
    const char* topic;
    const char* topic2;
    uint8_t* sendbuf;
    size_t sendbufsz;
    uint8_t* recvbuf;
    size_t recvbufsz;
};

char s_compare[1024] = "";

void reconnect_client(struct mqtt_client* client, void **reconnect_state_vptr);
void publish_callback(void** unused, struct mqtt_response_publish *published);
void* client_refresher(void* client);
void exit_example(int status, int sockfd, pthread_t *client_daemon);



/***
	IEC104 Server stuf
***/

static bool running = true;

void
sigint_handler(int signalId)
{
    running = false;
}

void
printCP56Time2a(CP56Time2a time)
{
    printf("%02i:%02i:%02i %02i/%02i/%04i", CP56Time2a_getHour(time),
                             CP56Time2a_getMinute(time),
                             CP56Time2a_getSecond(time),
                             CP56Time2a_getDayOfMonth(time),
                             CP56Time2a_getMonth(time),
                             CP56Time2a_getYear(time) + 2000);
}

/* Callback handler to log sent or received messages (optional) */
static void
rawMessageHandler(void* parameter, IMasterConnection conneciton, uint8_t* msg, int msgSize, bool sent)
{
    if (sent)
        printf("SEND: ");
    else
        printf("RCVD: ");

    int i;
    for (i = 0; i < msgSize; i++) {
        printf("%02x ", msg[i]);
    }

    printf("\n");
}

static bool
clockSyncHandler (void* parameter, IMasterConnection connection, CS101_ASDU asdu, CP56Time2a newTime)
{
    printf("Process time sync command with time "); printCP56Time2a(newTime); printf("\n");

    uint64_t newSystemTimeInMs = CP56Time2a_toMsTimestamp(newTime);

    /* Set time for ACT_CON message */
    CP56Time2a_setFromMsTimestamp(newTime, Hal_getTimeInMs());

    /* update system time here */

    return true;
}

static bool
interrogationHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu, uint8_t qoi)
{
    printf("Received interrogation for group %i\n", qoi);

    if (qoi == 20) { /* only handle station interrogation */

        CS101_AppLayerParameters alParams = IMasterConnection_getApplicationLayerParameters(connection);

        IMasterConnection_sendACT_CON(connection, asdu, false);

        /* The CS101 specification only allows information objects without timestamp in GI responses */

        CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);

        
		/* metingen tbv GI */
		InformationObject io = (InformationObject) MeasuredValueScaled_create(NULL, 100, -1, IEC60870_QUALITY_GOOD);

        CS101_ASDU_addInformationObject(newAsdu, io);
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) MeasuredValueScaled_create((MeasuredValueScaled) io, 101, 23, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) MeasuredValueScaled_create((MeasuredValueScaled) io, 102, 2300, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) MeasuredValueScaled_create((MeasuredValueScaled) io, 104, -2, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) MeasuredValueScaled_create((MeasuredValueScaled) io, 105, 6300, IEC60870_QUALITY_GOOD));
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) MeasuredValueScaled_create((MeasuredValueScaled) io, 106, 63, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);
        
		
        
		/*  2 singlepoint tbv GI
		newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_INTERROGATED_BY_STATION,
                    0, 1, false, false);

        io = (InformationObject) SinglePointInformation_create(NULL, 104, true, IEC60870_QUALITY_GOOD);

        CS101_ASDU_addInformationObject(newAsdu, io);

        CS101_ASDU_addInformationObject(newAsdu, (InformationObject)
            SinglePointInformation_create((SinglePointInformation) io, 105, false, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);
		*/
		
		bool value = false;

		newAsdu = CS101_ASDU_create(alParams, true, CS101_COT_INTERROGATED_BY_STATION, 0, 1, false, false);

        
     	//if(!strcmp(digitals[0],"true")) {value = true;}else{value = false;}
	value = true;
		io = (InformationObject) SinglePointInformation_create(NULL, 300, value, IEC60870_QUALITY_GOOD);
	    CS101_ASDU_addInformationObject(newAsdu, io);
     	//if(!strcmp(digitals[1],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 301, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[2],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 302, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[3],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 303, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[4],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 304, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[5],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 305, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[6],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 306, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[7],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 307, value, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        IMasterConnection_sendASDU(connection, newAsdu);

        CS101_ASDU_destroy(newAsdu);

        IMasterConnection_sendACT_TERM(connection, asdu);
    }
    else {
        IMasterConnection_sendACT_CON(connection, asdu, true);
    }

    return true;
}

static bool
asduHandler(void* parameter, IMasterConnection connection, CS101_ASDU asdu)
{
    if (CS101_ASDU_getTypeID(asdu) == C_SC_NA_1) {
        printf("received single command\n");

        if  (CS101_ASDU_getCOT(asdu) == CS101_COT_ACTIVATION) {
            InformationObject io = CS101_ASDU_getElement(asdu, 0);

            if (InformationObject_getObjectAddress(io) == 5000) {
                SingleCommand sc = (SingleCommand) io;

                printf("IOA: %i switch to %i\n", InformationObject_getObjectAddress(io),
                        SingleCommand_getState(sc));

                CS101_ASDU_setCOT(asdu, CS101_COT_ACTIVATION_CON);
            }
            else
                CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_IOA);

            InformationObject_destroy(io);
        }
        else
            CS101_ASDU_setCOT(asdu, CS101_COT_UNKNOWN_COT);

        IMasterConnection_sendASDU(connection, asdu);

        return true;
    }

    return false;
}

static bool
connectionRequestHandler(void* parameter, const char* ipAddress)
{
    printf("New connection request from %s\n", ipAddress);

#if 0
    if (strcmp(ipAddress, "192.168.1.85") == 0) {
        printf("Accept connection\n");
        return true;
    }
    else {
        printf("Deny connection\n");
        return false;
    }
#else
    return true;
#endif
}

static void
connectionEventHandler(void* parameter, IMasterConnection con, CS104_PeerConnectionEvent event)
{
    if (event == CS104_CON_EVENT_CONNECTION_OPENED) {
        printf("Connection opened (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_CONNECTION_CLOSED) {
        printf("Connection closed (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_ACTIVATED) {
        printf("Connection activated (%p)\n", con);
    }
    else if (event == CS104_CON_EVENT_DEACTIVATED) {
        printf("Connection deactivated (%p)\n", con);
    }
}

int
main(int argc, char** argv)
{
	/* MQTT stuf */
	const char* addr;
    const char* port;
    const char* topic;
    const char* topic2;
	addr   = "192.168.1.85";
	port   = "1883";
	topic  = "modbus/#";
	topic2 = "modbus2/#";
	
    /* build the reconnect_state structure which will be passed to reconnect */
    struct reconnect_state_t reconnect_state;
    reconnect_state.hostname = addr;
    reconnect_state.port = port;
    reconnect_state.topic = topic;
    reconnect_state.topic2 = topic2;
    uint8_t sendbuf[2048];
    uint8_t recvbuf[1024];
    reconnect_state.sendbuf = sendbuf;
    reconnect_state.sendbufsz = sizeof(sendbuf);
    reconnect_state.recvbuf = recvbuf;
    reconnect_state.recvbufsz = sizeof(recvbuf);

    /* setup a client */
    struct mqtt_client client;

    mqtt_init_reconnect(&client, 
                        reconnect_client, &reconnect_state, 
                        publish_callback
    );

    /* start a thread to refresh the client (handle egress and ingree client traffic) */
    pthread_t client_daemon;
    if(pthread_create(&client_daemon, NULL, client_refresher, &client)) {
        fprintf(stderr, "Failed to start client daemon.\n");
        exit_example(EXIT_FAILURE, -1, NULL);

    }

    /* start publishing the time */
    printf("listening for '%s' messages.\n",  topic);
    printf("and listening for '%s' messages.\n", topic2);
    printf("Press CTRL-D to exit.\n\n");


    /* End of MQTT stuf */	
	
	
	
	/* Start of IEC104 stuf */	
    /* Add Ctrl-C handler */
    signal(SIGINT, sigint_handler);

    /* create a new slave/server instance with default connection parameters and
     * default message queue size */
    CS104_Slave slave = CS104_Slave_create(100, 100);

    CS104_Slave_setLocalAddress(slave, "0.0.0.0");

    /* Set mode to a multiple redundancy groups
     * NOTE: library has to be compiled with CONFIG_CS104_SUPPORT_SERVER_MODE_SINGLE_REDUNDANCY_GROUP enabled (=1)
     */
    CS104_Slave_setServerMode(slave, CS104_MODE_MULTIPLE_REDUNDANCY_GROUPS);

    CS104_RedundancyGroup redGroup1 = CS104_RedundancyGroup_create("red-group-1");
    CS104_RedundancyGroup_addAllowedClient(redGroup1, "192.168.2.9");

    CS104_RedundancyGroup redGroup2 = CS104_RedundancyGroup_create("red-group-2");
    CS104_RedundancyGroup_addAllowedClient(redGroup2, "192.168.2.223");
    CS104_RedundancyGroup_addAllowedClient(redGroup2, "192.168.2.222");

    CS104_RedundancyGroup redGroup3 = CS104_RedundancyGroup_create("catch-all");

    CS104_Slave_addRedundancyGroup(slave, redGroup1);
    CS104_Slave_addRedundancyGroup(slave, redGroup2);
    CS104_Slave_addRedundancyGroup(slave, redGroup3);

    /* get the connection parameters - we need them to create correct ASDUs */
    CS101_AppLayerParameters alParams = CS104_Slave_getAppLayerParameters(slave);

    /* set the callback handler for the clock synchronization command */
    CS104_Slave_setClockSyncHandler(slave, clockSyncHandler, NULL);

    /* set the callback handler for the interrogation command */
    CS104_Slave_setInterrogationHandler(slave, interrogationHandler, NULL);

    /* set handler for other message types */
    CS104_Slave_setASDUHandler(slave, asduHandler, NULL);

    /* set handler to handle connection requests (optional) */
    CS104_Slave_setConnectionRequestHandler(slave, connectionRequestHandler, NULL);

    /* set handler to track connection events (optional) */
    CS104_Slave_setConnectionEventHandler(slave, connectionEventHandler, NULL);

    /* uncomment to log messages */
    CS104_Slave_setRawMessageHandler(slave, rawMessageHandler, NULL);

    CS104_Slave_start(slave);

    if (CS104_Slave_isRunning(slave) == false) {
        printf("Starting server failed!\n");
        goto exit_program;
    }

    int16_t scaledValue = 0;

    while (running) {

		Thread_sleep(1000);
        bool value = true;
		
		CS101_ASDU newAsdu = CS101_ASDU_create(alParams, false, CS101_COT_SPONTANEOUS, 0, 1, false, false);


		
		//if(!strcmp(digitals[0],"true")) {value = true;}else{value = false;}
	value = true;
		InformationObject io = (InformationObject) SinglePointInformation_create(NULL, 300, value, IEC60870_QUALITY_GOOD);
	    CS101_ASDU_addInformationObject(newAsdu, io);
     	//if(!strcmp(digitals[1],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 301, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[2],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 302, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[3],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 303, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[4],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 304, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[5],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 305, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[6],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 306, value, IEC60870_QUALITY_GOOD));
     	//if(!strcmp(digitals[7],"true")) {value = true;}else{value = false;}
        CS101_ASDU_addInformationObject(newAsdu, (InformationObject) SinglePointInformation_create((SinglePointInformation) io, 307, value, IEC60870_QUALITY_GOOD));

        InformationObject_destroy(io);

        /* Add ASDU to slave event queue - don't release the ASDU afterwards!
         * The ASDU will be released by the Slave instance when the ASDU
         * has been sent.
         */
        CS104_Slave_enqueueASDU(slave, newAsdu);

        CS101_ASDU_destroy(newAsdu);
	
	
	}

    CS104_Slave_stop(slave);

exit_program:
    CS104_Slave_destroy(slave);

    Thread_sleep(500);
	
	/* End of IEC104 stuf */
	
    /* disconnect from MQTT Broker*/
    printf("\n%s disconnecting from %s\n", argv[0], addr);
    sleep(1);
    exit_example(EXIT_SUCCESS, client.socketfd, &client_daemon);

	
	
}


/* Start of MQTT funtions */

void reconnect_client(struct mqtt_client* client, void **reconnect_state_vptr)
{
    struct reconnect_state_t *reconnect_state = *((struct reconnect_state_t**) reconnect_state_vptr);

    /* Close the clients socket if this isn't the initial reconnect call */
    if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
        close(client->socketfd);
    }

    /* Perform error handling here. */
    if (client->error != MQTT_ERROR_INITIAL_RECONNECT) {
        printf("reconnect_client: called while client was in error state \"%s\"\n", 
               mqtt_error_str(client->error)
        );
    }

    /* Open a new socket. */
    int sockfd = open_nb_socket(reconnect_state->hostname, reconnect_state->port);
    if (sockfd == -1) {
        perror("Failed to open socket: ");
        exit_example(EXIT_FAILURE, sockfd, NULL);
    }

    /* Reinitialize the client. */
    mqtt_reinit(client, sockfd, 
                reconnect_state->sendbuf, reconnect_state->sendbufsz,
                reconnect_state->recvbuf, reconnect_state->recvbufsz
    );
    
    /* Send connection request to the broker. */
    mqtt_connect(client, "subscribing_client", NULL, NULL, 0, NULL, NULL, 0, 400);

    /* Subscribe to the topic. */
    mqtt_subscribe(client, reconnect_state->topic, 0);
    mqtt_subscribe(client, reconnect_state->topic2, 0);
}


void exit_example(int status, int sockfd, pthread_t *client_daemon)
{
    if (sockfd != -1) close(sockfd);
    if (client_daemon != NULL) pthread_cancel(*client_daemon);
    exit(status);
}



void publish_callback(void** unused, struct mqtt_response_publish *published) 
{

    /* note that published->topic_name is NOT null-terminated (here we'll change it to a c-string) */
    char* topic_name = (char*) malloc(published->topic_name_size + 1);
    memcpy(topic_name, published->topic_name, published->topic_name_size);
    topic_name[published->topic_name_size] = '\0';


    char* s = (char*) malloc(published->application_message_size + 1);
    memcpy(s, published->application_message, published->application_message_size);
    s[published->application_message_size] = '\0';

    //printf("voor %s\n",s);

    if(strcmp( s_compare,s))
    { 
      char str_digitals[1024] = "";
      strcpy( str_digitals,s);
      int j,len=strlen(str_digitals); 
      for(j=1;j<len-1;j++)
      {
        str_digitals[j-1]=s[j];
      }
      str_digitals[j-1]='\0';
      //printf("Na %s\n",str_digitals);

      // remove the "
      int i = 0;
      for(i = 0; str_digitals[i] != '\0'; ++i)
      {
          while ( str_digitals[i] == '\"' || str_digitals[i] == ' ')
          {
              for(j = i; str_digitals[j] != '\0'; ++j)
              {
                  str_digitals[j] = str_digitals[j+1];
              }
              str_digitals[j] = '\0';
          }
      }

      //printf("Na Na %s\n",str_digitals);
 


      i = 0;
      char *p = strtok (str_digitals, ",");
      

      while (p != NULL)
      {
        digitals[i++] = p;

        p = strtok (NULL, ",");
      }

      for (i = 0; i < 8; ++i) 
      {
        printf("%s\n", digitals[i]);
      }
      //printf(" %lu %s %s\n",  (size_t) published->application_message_size, topic_name, s);
      //printf(" %s\n", s);
      printf("\n");
    }

    strcpy(s_compare,s);

    free(topic_name);
}


void* client_refresher(void* client)
{
    while(1) 
    {
        mqtt_sync((struct mqtt_client*) client);
        usleep(100000U);
    }
    return NULL;
}

