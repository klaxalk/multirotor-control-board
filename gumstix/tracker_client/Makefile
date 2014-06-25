#
# Date:      2011/07/11 17:55
# Author:    Jan Faigl
#

CXX:= $(CXX)

CXXFLAGS+=
LDFLAGS+=

OBJS=$(patsubst %.cc,%.o,$(wildcard *.cc))


OBJSC=serialization.o tracker_client.o
TARGET=tclient treader tclient_log tclient_capture


bin: $(TARGET)

$(OBJS): %.o: %.cc
	$(CXX) -c $< $(CXXFLAGS) $(CPPFLAGS) -o $@

tclient: $(OBJS)
	$(CXX) tclient.o $(OBJSC) $(LDFLAGS) -o $@

treader: $(OBJS)
	$(CXX) treader.o $(OBJSC) $(LDFLAGS) -o $@

tclient_log: $(OBJS)
	$(CXX) tclient_log.o $(OBJSC) $(LDFLAGS) -o $@

tclient_capture: $(OBJS)
	$(CXX) tclient_capture.o $(OBJSC) $(LDFLAGS) -o $@

clean:
	$(RM) $(OBJS) $(TARGET)

