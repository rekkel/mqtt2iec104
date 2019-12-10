FROM inovium/ir1101_gcc:1.0 as dev-container

WORKDIR /usr/src/mqtt2iec104
COPY cs104_redundancy_server.c ./
COPY Makefile ./
COPY libs/ libs/

RUN make

FROM devhub-docker.cisco.com/iox-docker/ir1101/base-rootfs
WORKDIR /usr/mqtt2iec104/
COPY --from=dev-container /usr/src/mqtt2iec104/cs104_redundancy_server .
#CMD ["./cs104_redundancy_server"]
