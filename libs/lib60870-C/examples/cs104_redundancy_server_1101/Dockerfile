FROM debian

LABEL MAINTAINER="Rick Ekkelboom <rick.ekkelboom@alliander.com>" \
      LOGGING='yes'

WORKDIR /usr/src/app

COPY ./cs104_redundancy_server .

# The directory to write the files to with a trailing slash. Default ./
ENV DESTINATION         ./

# The name and version that will be reported in the application and monitoring
ENV APPLICATION_NAME    MQTT_IEC104
ENV APPLICATION_VERSION 0.0.1

CMD ["./cs104_redundancy_server"]

