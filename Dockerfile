FROM ubuntu:latest

WORKDIR /usr/src/mqtt2iec104

COPY cs104_redundancy_server ./

CMD ./cs104_redundancy_server
