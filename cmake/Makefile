TARGET = ../output/xunyi-controlled-exe


OBJECTS = main.o DJI_Pro_App.o DJI_Pro_Hw.o DJI_Pro_Link.o DJI_Pro_Sample.o DJI_Pro_Codec.o DJI_Pro_Rmu.o tinyxml2.o uart_common.o wireless_debug.o route.o control_law.o control_steer.o range.o fisheye.o getTargetLoc.o inifile.o image_identify.o thread_common_op.o status_led.o sd_store_log.o http_chat_3g.o cJSON.o ultra_filter.o

CFLAGS = -Wall -O3 -DPLATFORM_LINUX -DTINYXML_CFG -Isrc/ -I/src/DJI_LIB -I/src/XY_LIB -I/src/XY_LIB/common -I/usr/local/include/opencv -I/usr/local/include

LDFLAGS = -lpthread -lm /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_contrib.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_gpu.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_legacy.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_nonfree.so /usr/local/lib/libopencv_objdetect.so /usr/local/lib/libopencv_ocl.so /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_superres.so /usr/local/lib/libopencv_ts.a /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videostab.so -lrt -ldl 


CXX = g++ -pg -g -rdynamic

$(TARGET) : $(OBJECTS)
	$(CXX) -o $(TARGET) $(OBJECTS) $(LDFLAGS)
	
main.o : ../src/main.cpp
	$(CXX) $(CFLAGS) -c ../src/main.cpp

DJI_Pro_App.o : ../src/DJI_LIB/DJI_Pro_App.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_App.cpp
	
DJI_Pro_Hw.o : ../src/DJI_LIB/DJI_Pro_Hw.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Hw.cpp

DJI_Pro_Link.o : ../src/DJI_LIB/DJI_Pro_Link.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Link.cpp

DJI_Pro_Sample.o : ../src/DJI_Pro_Sample.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_Pro_Sample.cpp
	
DJI_Pro_Codec.o : ../src/DJI_LIB/DJI_Pro_Codec.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Codec.cpp

DJI_Pro_Rmu.o: ../src/DJI_LIB/DJI_Pro_Rmu.cpp
	$(CXX) $(CFLAGS) -c ../src/DJI_LIB/DJI_Pro_Rmu.cpp

tinyxml2.o : ../src/tinyxml2.cpp
	$(CXX) $(CFLAGS) -c ../src/tinyxml2.cpp
	
uart_common.o : ../src/XY_LIB/common/uart_common.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/common/uart_common.cpp

wireless_debug.o : ../src/XY_LIB/wireless_debug.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/wireless_debug.cpp
	
route.o : ../src/XY_LIB/route.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/route.cpp
	
control_law.o : ../src/XY_LIB/control_law.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/control_law.cpp
	
control_steer.o : ../src/XY_LIB/control_steer.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/control_steer.cpp
	
range.o : ../src/XY_LIB/range.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/range.cpp
	
fisheye.o : ../src/XY_LIB/fisheye.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/fisheye.cpp
	
getTargetLoc.o : ../src/XY_LIB/getTargetLoc.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/getTargetLoc.cpp
	
inifile.o : ../src/XY_LIB/inifile.c
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/inifile.c
	
image_identify.o : ../src/XY_LIB/image_identify.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/image_identify.cpp
	
thread_common_op.o : ../src/XY_LIB/thread_common_op.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/thread_common_op.cpp
	
status_led.o : ../src/XY_LIB/status_led.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/status_led.cpp
	
http_chat_3g.o : ../src/XY_LIB/http_chat_3g.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/http_chat_3g.cpp

cJSON.o : ../src/XY_LIB/cJSON.c
	gcc $(CFLAGS) -c ../src/XY_LIB/cJSON.c
	
sd_store_log.o : ../src/XY_LIB/sd_store_log.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/sd_store_log.cpp

ultra_filter.o : ../src/XY_LIB/ultra_filter.cpp
	$(CXX) $(CFLAGS) -c ../src/XY_LIB/ultra_filter.cpp
	
clean:
	-rm *.o
	-rm $(TARGET)
