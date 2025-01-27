HIDAPI_HEADER_PATH = /usr/local/include/hidapi

CC=gcc
IFLAGS=-I$(HIDAPI_HEADER_PATH)
LDLIBS=-lhidapi
CFLAGS = -Wall -Wextra -O2 -g $(IFLAGS)

BUILDDIR = ./build
DEPDIR   = ./.dep

SRCS = $(wildcard *.c)
OBJS = $(SRCS:%.c=$(BUILDDIR)/%.o)
EXE  = $(BUILDDIR)/test-usb-hid

all: $(EXE)

$(EXE): $(OBJS)

-include $(subst .c,.d,$(SRCS))

$(BUILDDIR)/%.o: %.o
	mv $^ $@

$(DEPDIR)/%.d: %.c
	$(CC) $(CFLAGS) $(CPPFLAGS) $(TARGET_ARCH) -MM -MF $@ -MP -MT $(subst .c,.o,$<) $<

clean:
	rm -f $(EXE)
	rm -f $(OBJS)
	rm -f $(subst .c,.d,$(SRCS))
	rm -f *~

.PHONY: clean all
