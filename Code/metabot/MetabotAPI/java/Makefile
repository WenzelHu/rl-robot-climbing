
INCLUDE = -I ../src -I ../src/vendor/serial/include
CXXFLAGS = -Wall $(INCLUDE)

.Phony: clean test all-demo

all: libholojava.jnilib HoloBinding.h all-demo

HoloBinding.class: HoloBinding.java
	javac HoloBinding.java

HoloBinding.h: HoloBinding.java HoloBinding.class
	javah -jni HoloBinding

HoloBinding.o: HoloBinding.h HoloBinding.cpp
	g++ $(CXXFLAGS) -I /System/Library/Frameworks/JavaVM.framework/Versions/Current/Headers -c HoloBinding.cpp

libholojava.jnilib: HoloBinding.o
	g++ $(CXXFLAGS) -dynamiclib -o libholojava.jnilib HoloBinding.o -L../src/build -L../src/build/serial -lLibMetabot -lserial

test: HoloBindingTest.java HoloBinding.class libholojava.jnilib
	javac -classpath . HoloBindingTest.java
	java HoloBindingTest

all-demo: 
	javac -classpath . DemoHoloMonitor.java
	javac -classpath . DemoHoloState.java
	javac -classpath . DemoHoloMotion.java
	javac -classpath . DemoHoloLine.java
	javac -classpath . DemoHoloPushObstacle.java
	javac -classpath . DebugHolo.java
clean:
	rm -f libholojava.jnilib HoloBinding.o HoloBinding.h *.class 
