PROTO_PATH=../src/proto

packet_pb2.py: $(PROTO_PATH)/packet.proto
	protoc --python_out=. --proto_path=$(PROTO_PATH) $<
