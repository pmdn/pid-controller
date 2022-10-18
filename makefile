ifeq ($(OS),Windows_NT)
  ifeq ($(shell uname -s),) # not in a bash-like shell
	CLEANUP = del /F /Q
	MKDIR = mkdir
  else # in a bash-like shell, like msys
	CLEANUP = rm -f
	MKDIR = mkdir -p
  endif
	TARGET_EXTENSION=exe
else
	CLEANUP = rm -f
	MKDIR = mkdir -p
	TARGET_EXTENSION=out
endif

.PHONY: clean
.PHONY: test

PATHU = test/unity/
PATHS = src/
PATHL = lib/
PATHI = include/
PATHT = test/
PATHB = build/
PATHD = build/depends/
PATHO = build/objs/
PATHR = build/results/

BUILD_PATHS = $(PATHB) $(PATHD) $(PATHO) $(PATHR)
SOURCE_PATHS = $(PATHU) $(PATHS) $(PATHL) $(PATHT)
INCLUDE_PATHS = $(PATHU) $(PATHS) $(PATHI) $(PATHL)
# Add this list to VPATH, the place make will look for the source files
VPATH = $(SOURCE_PATHS)

SRCT = $(wildcard $(PATHT)*.c)
SOURCES = $(foreach dir,$(SOURCE_PATHS),$(wildcard $(dir)/*.c))
SRCS = $(notdir $(filter %.c,$(SOURCES)))
OBJS = $(patsubst %.c,%.o,$(SRCS))
OBJECTS = $(addprefix $(PATHO),$(OBJS))
INCDIR = $(addprefix -I,$(INCLUDE_PATHS))
DEPFILES := $(SRCS:%.c=$(PATHD)%.d)


CC=gcc
COMPILE=$(CC) -c
LINK=$(CC)
OPT=-O0
DEPEND=$(CC) -MM -MG -MF
# generate files that encode make rules for the .h dependencies
DEPFLAGS=-MP -MD -MF $(PATHD)$*.d
CFLAGS=-Wall -Wextra -g -I. $(INCDIR) $(OPT) $(DEPFLAGS) -DTEST

RESULTS = $(patsubst $(PATHT)test_%.c,$(PATHR)test_%.txt,$(SRCT) )

PASSED = `grep -s PASS $(PATHR)*.txt`
FAIL = `grep -s FAIL $(PATHR)*.txt`
IGNORE = `grep -s IGNORE $(PATHR)*.txt`
SUMMARY = `grep -A2 "\-\-" $(PATHR)*.txt`

test: $(BUILD_PATHS) $(RESULTS)
	@echo "=============\nRUNNING TESTS:\n============="
	@echo "-----------------------\nIGNORES:\n-----------------------"
	@echo "$(IGNORE)"
	@echo "-----------------------\nFAILURES:\n-----------------------"
	@echo "$(FAIL)"
	@echo "-----------------------\nPASSED:\n-----------------------"
	@echo "$(PASSED)"
	@echo "-----------------------\nSUMMARY:"
	@echo "$(SUMMARY)"
	@echo "\nDONE"

$(PATHR)%.txt: $(PATHB)%.$(TARGET_EXTENSION)
	-./$< > $@ 2>&1

$(PATHB)test_%.$(TARGET_EXTENSION): $(OBJECTS)
	$(LINK) -o $@ $^

$(PATHO)%.o: %.c
$(PATHO)%.o: %.c $(PATHD)%.d
	$(COMPILE) $(CFLAGS) $< -o $@ 



$(PATHB):
	$(MKDIR) $(PATHB)

$(PATHD):
	$(MKDIR) $(PATHD)

$(PATHO):
	$(MKDIR) $(PATHO)

$(PATHR):
	$(MKDIR) $(PATHR)

# Guarantee that *.o files will be generated even when %.d files were not yet created
$(PATHD)%.d: ;

clean:
	$(CLEANUP) $(PATHO)*.o
	$(CLEANUP) $(PATHd)*.d
	$(CLEANUP) $(PATHB)*.$(TARGET_EXTENSION)
	$(CLEANUP) $(PATHR)*.txt
	@echo $(DEPFILES)

all: clean test

.PRECIOUS: $(PATHB)test_%.$(TARGET_EXTENSION)
.PRECIOUS: $(PATHD)%.d
.PRECIOUS: $(PATHO)%.o
.PRECIOUS: $(PATHR)%.txt

-include $(DEPFILES)
