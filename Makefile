RM=rm
MKDIR=mkdir
CC=gcc
STRIP=strip

OUTDIR?=build
OUTFILE="$(OUTDIR)/rtl_wmbus"
CFLAGS+=-Iinclude -std=gnu99
CFLAGS_WARNINGS?=-Wall -W -Waggregate-return -Wbad-function-cast -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wno-float-equal -Winline -Wmain -Wmissing-noreturn -Wno-missing-prototypes -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wshadow -Wsign-compare -Wstrict-prototypes -Wswitch -Wunreachable-code -Wno-unused -Wuninitialized
LIB?=-lm
SRC=rtl_wmbus.c
PYTHON?=python3

.PHONY: all release debug pi1 rebuild install clean test check test-unit test-captures test-runner

$(shell $(MKDIR) -p $(OUTDIR))

# Create a version number based on the latest git tag.
COMMIT_HASH?=$(shell git log --pretty=format:'%H' -n 1)
TAG?=$(shell git describe --tags --always)
BRANCH?=$(shell git rev-parse --abbrev-ref HEAD)
CHANGES?=$(shell git status -s | grep -v '?? ')

# Prefix with any development branch.
ifeq ($(BRANCH),master)
  BRANCH:=
else
  BRANCH:=$(BRANCH)_
endif

# The version is the git tag or tag-N-hash if there are N commits after the tag.
VERSION:=$(BRANCH)$(TAG)

ifneq ($(strip $(CHANGES)),)
  # There are local non-committed changes! Add this to the version string as well!
  VERSION:=$(VERSION) with local changes
  COMMIT_HASH:=$(COMMIT_HASH) with local changes
endif

$(shell echo "#define VERSION \"$(VERSION)\"" > $(OUTDIR)/version.h.tmp)
$(shell echo "#define COMMIT \"$(COMMIT_HASH)\"" >> $(OUTDIR)/version.h.tmp)

PREV_VERSION=$(shell cat -n $(OUTDIR)/version.h 2> /dev/null)
CURR_VERSION=$(shell cat -n $(OUTDIR)/version.h.tmp 2>/dev/null)
ifneq ($(PREV_VERSION),$(CURR_VERSION))
$(shell mv $(OUTDIR)/version.h.tmp $(OUTDIR)/version.h)
else
$(shell rm $(OUTDIR)/version.h.tmp)
endif

$(info Building $(VERSION))

all: release

release:
	$(CC) -DNDEBUG -O3                  $(CFLAGS) $(CFLAGS_WARNINGS) -o $(OUTFILE) $(SRC) $(LIB)

debug:
	$(CC) -DDEBUG  -O0 -g3 -ggdb -p -pg $(CFLAGS) $(CFLAGS_WARNINGS) -o $(OUTFILE) $(SRC) $(LIB)

test: release
	$(MAKE) check

# Check the existing binary so debug and sanitizer builds can use the same tests.
check: test-unit test-captures test-runner

test-unit:
	$(CC) -O0 -g $(CFLAGS) $(CFLAGS_WARNINGS) -o "$(OUTDIR)/test_core" tests/test_core.c $(LIB)
	"$(OUTDIR)/test_core"

test-captures:
	$(PYTHON) tests/replay_captures.py --binary $(OUTFILE)

test-runner:
	$(PYTHON) -m unittest discover -s tests -p 'test_*.py'

# Will build on Raspberry Pi 1 only
pi1:
	$(CC) -DNDEBUG -O3 -march=armv6 -mtune=arm1176jzf-s -mfloat-abi=hard -mfpu=vfp -ffast-math $(CFLAGS) $(CFLAGS_WARNINGS) -o $(OUTFILE) $(SRC) $(LIB)

rebuild: clean all

install:
	@if [ ! -f $(OUTFILE) ] ; then echo "Cannot find the binary to install! You have to run just \"make\" first!" ; exit 1 ; fi
	install -d $(DESTDIR)/usr/bin
	install $(OUTFILE) $(DESTDIR)/usr/bin

clean:
	$(RM) -rf "$(OUTDIR)"
