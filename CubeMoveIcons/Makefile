CXXFLAGS =	-O2 -Wall -fmessage-length=0

OBJS =		MoveIcons.o

LIBS =		-lgdi32

TARGET =	MoveIcons.exe

$(TARGET):	$(OBJS)
	$(CXX) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(OBJS:%.o=%.dep) $(TARGET)

%.dep : %.cpp
	g++ -M $(FLAGS) $< > $@

include $(OBJS:%.o=%.dep)
