LIB60870_HOME=libs/lib60870-C
MQTT_C_HOME=libs/MQTT-C-master

PROJECT_BINARY_NAME = cs104_redundancy_server
MQTT_C_SOURCES = ${MQTT_C_HOME}/src/mqtt.c ${MQTT_C_HOME}/src/mqtt_pal.c
PROJECT_SOURCES = cs104_redundancy_server.c $(MQTT_C_SOURCES)

include $(LIB60870_HOME)/make/target_system.mk
include $(LIB60870_HOME)/make/stack_includes.mk

INCLUDES += -I$(MQTT_C_HOME)/include
INCLUDES += -I$(MQTT_C_HOME)/examples/templates




all:	$(PROJECT_BINARY_NAME)

include $(LIB60870_HOME)/make/common_targets.mk


$(PROJECT_BINARY_NAME):	$(PROJECT_SOURCES) $(LIB_NAME)
	$(CC) $(CFLAGS) $(LDFLAGS) -g -o $(PROJECT_BINARY_NAME) $(PROJECT_SOURCES) $(INCLUDES) $(LIB_NAME) $(LDLIBS)

clean:
	rm -f $(PROJECT_BINARY_NAME)
	rm -rf ${LIB60870_HOME}/build 

