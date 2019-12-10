LIB60870_HOME=../..
MQTT_C_HOME=../../../..

PROJECT_BINARY_NAME = cs104_redundancy_server
MQTT_C_SOURCES = ../../../../MQTT-C-master/src/mqtt.c ../../../../MQTT-C-master/src/mqtt_pal.c
PROJECT_SOURCES = cs104_redundancy_server.c $(MQTT_C_SOURCES)

include $(LIB60870_HOME)/make/target_system.mk
include $(LIB60870_HOME)/make/stack_includes.mk



all:	$(PROJECT_BINARY_NAME)

include $(LIB60870_HOME)/make/common_targets.mk


$(PROJECT_BINARY_NAME):	$(PROJECT_SOURCES) $(LIB_NAME)
	$(CC) $(CFLAGS) $(LDFLAGS) -g -o $(PROJECT_BINARY_NAME) $(PROJECT_SOURCES) $(INCLUDES) $(LIB_NAME) $(LDLIBS)

clean:
	rm -f $(PROJECT_BINARY_NAME)

