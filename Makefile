#
# OMNeT++/OMNEST Makefile for callsimulation
#
# This file was generated with the command:
#  opp_makemake -f --deep -O out
#

# Name of target to be created (-o option)
TARGET = callsimulation$(EXE_SUFFIX)

# User interface (uncomment one) (-u option)
USERIF_LIBS = $(ALL_ENV_LIBS) # that is, $(TKENV_LIBS) $(QTENV_LIBS) $(CMDENV_LIBS)
#USERIF_LIBS = $(CMDENV_LIBS)
#USERIF_LIBS = $(TKENV_LIBS)
#USERIF_LIBS = $(QTENV_LIBS)

# C++ include paths (with -I)
INCLUDE_PATH = \
    -I. \
    -Ibuilder \
    -Inetworks \
    -Inode \
    -Iparsim \
    -Itemplates \
    -Itemplates/randomtopo

# Additional object and library files to link with
EXTRA_OBJS =

# Additional libraries (-L, -l options)
LIBS =

# Output directory
PROJECT_OUTPUT_DIR = out
PROJECTRELATIVE_PATH =
O = $(PROJECT_OUTPUT_DIR)/$(CONFIGNAME)/$(PROJECTRELATIVE_PATH)

# Object files for local .cc, .msg and .sm files
OBJS = \
    $O/builder/netbuilder.o \
    $O/node/L2Queue.o \
    $O/node/DijkstraFuzzy.o \
    $O/node/FlowRouting.o \
    $O/node/DijktraKShortest.o \
    $O/node/CallApp.o \
    $O/node/failureModule.o \
    $O/node/Packet_m.o

# Message files
MSGFILES = \
    node/Packet.msg

# SM files
SMFILES =

#------------------------------------------------------------------------------

# Pull in OMNeT++ configuration (Makefile.inc or configuser.vc)

ifneq ("$(OMNETPP_CONFIGFILE)","")
CONFIGFILE = $(OMNETPP_CONFIGFILE)
else
ifneq ("$(OMNETPP_ROOT)","")
CONFIGFILE = $(OMNETPP_ROOT)/Makefile.inc
else
CONFIGFILE = $(shell opp_configfilepath)
endif
endif

ifeq ("$(wildcard $(CONFIGFILE))","")
$(error Config file '$(CONFIGFILE)' does not exist -- add the OMNeT++ bin directory to the path so that opp_configfilepath can be found, or set the OMNETPP_CONFIGFILE variable to point to Makefile.inc)
endif

include $(CONFIGFILE)

# Simulation kernel and user interface libraries
OMNETPP_LIB_SUBDIR = $(OMNETPP_LIB_DIR)/$(TOOLCHAIN_NAME)
OMNETPP_LIBS = -L"$(OMNETPP_LIB_SUBDIR)" -L"$(OMNETPP_LIB_DIR)" -loppmain$D $(USERIF_LIBS) $(KERNEL_LIBS) $(SYS_LIBS)

COPTS = $(CFLAGS)  $(INCLUDE_PATH) -I$(OMNETPP_INCL_DIR)
MSGCOPTS = $(INCLUDE_PATH)
SMCOPTS =

# we want to recompile everything if COPTS changes,
# so we store COPTS into $COPTS_FILE and have object
# files depend on it (except when "make depend" was called)
COPTS_FILE = $O/.last-copts
ifneq ($(MAKECMDGOALS),depend)
ifneq ("$(COPTS)","$(shell cat $(COPTS_FILE) 2>/dev/null || echo '')")
$(shell $(MKPATH) "$O" && echo "$(COPTS)" >$(COPTS_FILE))
endif
endif

#------------------------------------------------------------------------------
# User-supplied makefile fragment(s)
# >>>
# <<<
#------------------------------------------------------------------------------

# Main target
all: $O/$(TARGET)
	$(Q)$(LN) $O/$(TARGET) .

$O/$(TARGET): $(OBJS)  $(wildcard $(EXTRA_OBJS)) Makefile
	@$(MKPATH) $O
	@echo Creating executable: $@
	$(Q)$(CXX) $(LDFLAGS) -o $O/$(TARGET)  $(OBJS) $(EXTRA_OBJS) $(AS_NEEDED_OFF) $(WHOLE_ARCHIVE_ON) $(LIBS) $(WHOLE_ARCHIVE_OFF) $(OMNETPP_LIBS)

.PHONY: all clean cleanall depend msgheaders smheaders

.SUFFIXES: .cc

$O/%.o: %.cc $(COPTS_FILE)
	@$(MKPATH) $(dir $@)
	$(qecho) "$<"
	$(Q)$(CXX) -c $(CXXFLAGS) $(COPTS) -o $@ $<

%_m.cc %_m.h: %.msg
	$(qecho) MSGC: $<
	$(Q)$(MSGC) -s _m.cc $(MSGCOPTS) $?

%_sm.cc %_sm.h: %.sm
	$(qecho) SMC: $<
	$(Q)$(SMC) -c++ -suffix cc $(SMCOPTS) $?

msgheaders: $(MSGFILES:.msg=_m.h)

smheaders: $(SMFILES:.sm=_sm.h)

clean:
	$(qecho) Cleaning...
	$(Q)-rm -rf $O
	$(Q)-rm -f callsimulation callsimulation.exe libcallsimulation.so libcallsimulation.a libcallsimulation.dll libcallsimulation.dylib
	$(Q)-rm -f ./*_m.cc ./*_m.h ./*_sm.cc ./*_sm.h
	$(Q)-rm -f builder/*_m.cc builder/*_m.h builder/*_sm.cc builder/*_sm.h
	$(Q)-rm -f networks/*_m.cc networks/*_m.h networks/*_sm.cc networks/*_sm.h
	$(Q)-rm -f node/*_m.cc node/*_m.h node/*_sm.cc node/*_sm.h
	$(Q)-rm -f parsim/*_m.cc parsim/*_m.h parsim/*_sm.cc parsim/*_sm.h
	$(Q)-rm -f templates/*_m.cc templates/*_m.h templates/*_sm.cc templates/*_sm.h
	$(Q)-rm -f templates/randomtopo/*_m.cc templates/randomtopo/*_m.h templates/randomtopo/*_sm.cc templates/randomtopo/*_sm.h

cleanall: clean
	$(Q)-rm -rf $(PROJECT_OUTPUT_DIR)

depend:
	$(qecho) Creating dependencies...
	$(Q)$(MAKEDEPEND) $(INCLUDE_PATH) -f Makefile -P\$$O/ -- $(MSG_CC_FILES) $(SM_CC_FILES)  ./*.cc builder/*.cc networks/*.cc node/*.cc parsim/*.cc templates/*.cc templates/randomtopo/*.cc

# DO NOT DELETE THIS LINE -- make depend depends on it.
$O/builder/netbuilder.o: builder/netbuilder.cc
$O/node/CallApp.o: node/CallApp.cc \
	node/CallApp.h \
	node/DijkstraFuzzy.h \
	node/Packet_m.h
$O/node/DijkstraFuzzy.o: node/DijkstraFuzzy.cc \
	node/DijkstraFuzzy.h
$O/node/DijktraKShortest.o: node/DijktraKShortest.cc \
	node/DijktraKShortest.h
$O/node/FlowRouting.o: node/FlowRouting.cc \
	node/DijktraKShortest.h \
	node/FailureEvent.h \
	node/FlowRouting.h \
	node/Packet_m.h
$O/node/L2Queue.o: node/L2Queue.cc
$O/node/Packet_m.o: node/Packet_m.cc \
	node/Packet_m.h
$O/node/failureModule.o: node/failureModule.cc \
	node/FailureEvent.h \
	node/failureModule.h

