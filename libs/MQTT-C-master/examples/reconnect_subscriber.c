
/**
 * @file
 * A simple subscriber program that performs automatic reconnections.
 */
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <mqtt.h>
#include "templates/posix_sockets.h"

/**
 * @brief A structure that I will use to keep track of some data needed 
 *        to setup the connection to the broker.
 * 
 * An instance of this struct will be created in my \c main(). Then, whenever
 * \ref reconnect_client is called, this instance will be passed. 
 */
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


/**
 * @brief My reconnect callback. It will reestablish the connection whenever 
 *        an error occurs. 
 */
void reconnect_client(struct mqtt_client* client, void **reconnect_state_vptr);

/**
 * @brief The function will be called whenever a PUBLISH message is received.
 */
void publish_callback(void** unused, struct mqtt_response_publish *published);

/**
 * @brief The client's refresher. This function triggers back-end routines to 
 *        handle ingress/egress traffic to the broker.
 * 
 * @note All this function needs to do is call \ref __mqtt_recv and 
 *       \ref __mqtt_send every so often. I've picked 100 ms meaning that 
 *       client ingress/egress traffic will be handled every 100 ms.
 */
void* client_refresher(void* client);

/**
 * @brief Safelty closes the \p sockfd and cancels the \p client_daemon before \c exit. 
 */
void exit_example(int status, int sockfd, pthread_t *client_daemon);


int main(int argc, const char *argv[]) 
{
    const char* addr;
    const char* port;
    const char* topic;
    const char* topic2;

    /* get address (argv[1] if present) */
    if (argc > 1) {
        addr = argv[1];
    } else {
        addr = "127.0.0.1";
    }

    /* get port number (argv[2] if present) */
    if (argc > 2) {
        port = argv[2];
    } else {
        port = "1883";
    }

    /* get the topic name to publish */
    if (argc > 3) {
        topic = argv[3];
    } else {
        topic = "modbus/#";
        topic2 = "modbus2/#";
    }

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
    printf("Press ENTER to inject an error.\n");
    printf("Press CTRL-D to exit.\n\n");
    
    /* block */
    while(fgetc(stdin) != EOF) {
        printf("Injecting error: \"MQTT_ERROR_SOCKET_ERROR\"\n");
        client.error = MQTT_ERROR_SOCKET_ERROR;
    } 
    
    /* disconnect */
    printf("\n%s disconnecting from %s\n", argv[0], addr);
    sleep(1);

    /* exit */ 
    exit_example(EXIT_SUCCESS, client.socketfd, &client_daemon);
}

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
      char *digitals[8];

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
