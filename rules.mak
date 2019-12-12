
# These are used when we want to do substitutions without confusing Make
NULL  :=
SPACE := $(NULL) #
COMMA := ,

# Don't use implicit rules or variables
# we have explicit rules for everything
MAKEFLAGS += -rR

# Files with this suffixes are final, don't try to generate them
# using implicit rules
%/trace-events:
%.hx:
%.py:
%.objs:
%.d:
%.h:
%.c:
%.cc:
%.cpp:
%.m:
%.mak:
clean-target:

# Flags for dependency generation
QEMU_DGFLAGS += -MMD -MP -MT $@ -MF $(@D)/$(*F).d

# Compiler searches the source file dir first, but in vpath builds
# we need to make it search the build dir too, before any other
# explicit search paths. There are two search locations in the build
# dir, one absolute and the other relative to the compiler working
# directory. These are the same for target-independent files, but
# different for target-dependent ones.
QEMU_LOCAL_INCLUDES = -iquote $(BUILD_DIR) -iquote $(BUILD_DIR)/$(@D) -iquote $(@D)

WL_U := -Wl,-u,
find-symbols = $(if $1, $(sort $(shell $(NM) -P -g $1 | $2)))
defined-symbols = $(call find-symbols,$1,awk '$$2!="U"{print $$1}')
undefined-symbols = $(call find-symbols,$1,awk '$$2=="U"{print $$1}')

WL := -Wl,
ifdef CONFIG_DARWIN
whole-archive = $(WL)-force_load,$1
else
whole-archive = $(WL)--whole-archive $1 $(WL)--no-whole-archive
endif

# All the .mo objects in -m variables are also added into corresponding -y
# variable in unnest-vars, but filtered out here, when LINK is called.
#
# The .mo objects are supposed to be linked as a DSO, for module build. So here
# they are only used as a placeholders to generate those "archive undefined"
# symbol options (-Wl,-u,$symbol_name), which are the archive functions
# referenced by the code in the DSO.
#
# Also the presence in -y variables will also guarantee they are built before
# linking executables that will load them. So we can look up symbol reference
# in LINK.
#
# This is necessary because the exectuable itself may not use the function, in
# which case the function would not be linked in. Then the DSO loading will
# fail because of the missing symbol.
process-archive-undefs = $(filter-out %.a %.fa %.mo %$(DSOSUF),$1) \
                $(addprefix $(WL_U), \
                     $(filter $(call defined-symbols,$(filter %.a %.fa, $1)), \
		$(call undefined-symbols,$(filter %.mo %$(DSOSUF),$1)))) \
		$(foreach l,$(filter %.fa,$1),$(call whole-archive,$l)) \
		$(filter %.a,$1)

extract-libs = $(strip $(foreach o,$(filter-out %.mo %$(DSOSUF),$1),$($o-libs)))
expand-objs = $(strip $(sort $(filter %.o,$1)) \
		$(foreach o,$(filter %.mo %$(DSOSUF),$1),$($o-objs)) \
		$(filter-out %.o %.mo %$(DSOSUF),$1))

%.o: %.c
	@mkdir -p $(dir $@)
	$(call quiet-command,$(CC) $(QEMU_LOCAL_INCLUDES) $(QEMU_INCLUDES) \
	       $(QEMU_CFLAGS) $(QEMU_DGFLAGS) $(CFLAGS) $($@-cflags) \
	       -c -o $@ $<,"CC","$(TARGET_DIR)$@")

# If we have a CXX we might have some C++ objects, in which case we
# must link with the C++ compiler, not the plain C compiler.
LINKPROG = $(or $(CXX),$(CC))

LINK = $(call quiet-command, $(LINKPROG) $(QEMU_LDFLAGS) $(QEMU_CFLAGS) $(CFLAGS) $(LDFLAGS) -o $@ \
       $(call process-archive-undefs, $1) \
       $(call extract-libs,$1) $(LIBS),"LINK","$(TARGET_DIR)$@")

%.o: %.S
	$(call quiet-command,$(CCAS) $(QEMU_LOCAL_INCLUDES) $(QEMU_INCLUDES) \
	       $(QEMU_CFLAGS) $(QEMU_DGFLAGS) $(CFLAGS) \
	       -c -o $@ $<,"CCAS","$(TARGET_DIR)$@")

%.o: %.cc
	$(call quiet-command,$(CXX) $(QEMU_LOCAL_INCLUDES) $(QEMU_INCLUDES) \
	       $(QEMU_CXXFLAGS) $(QEMU_DGFLAGS) $(CFLAGS) $($@-cflags) \
	       -c -o $@ $<,"CXX","$(TARGET_DIR)$@")

%.o: %.cpp
	$(call quiet-command,$(CXX) $(QEMU_LOCAL_INCLUDES) $(QEMU_INCLUDES) \
	       $(QEMU_CXXFLAGS) $(QEMU_DGFLAGS) $(CFLAGS) $($@-cflags) \
	       -c -o $@ $<,"CXX","$(TARGET_DIR)$@")

%.o: %.m
	$(call quiet-command,$(OBJCC) $(QEMU_LOCAL_INCLUDES) $(QEMU_INCLUDES) \
	       $(QEMU_CFLAGS) $(QEMU_DGFLAGS) $(CFLAGS) $($@-cflags) \
	       -c -o $@ $<,"OBJC","$(TARGET_DIR)$@")

%.o: %.dtrace
	$(call quiet-command,dtrace -o $@ -G -s $<,"GEN","$(TARGET_DIR)$@")

DSO_OBJ_CFLAGS := -fPIC -DBUILD_DSO
module-common.o: CFLAGS += $(DSO_OBJ_CFLAGS)
%$(DSOSUF): LDFLAGS += $(LDFLAGS_SHARED)
%$(DSOSUF): %.mo
	$(call LINK,$^)
	@# Copy to build root so modules can be loaded when program started without install
	$(if $(findstring /,$@),$(call quiet-command,cp $@ $(subst /,-,$@),"CP","$(subst /,-,$@)"))


LD_REL := $(CC) -nostdlib $(LD_REL_FLAGS)

%.mo:
	$(call quiet-command,$(LD_REL) -o $@ $^,"LD","$(TARGET_DIR)$@")

.PHONY: modules
modules:

%$(EXESUF): %.o
	$(call LINK,$(filter %.o %.a %.mo %.fa, $^))

%.a:
	$(call quiet-command,rm -f $@ && $(AR) rcs $@ $^,"AR","$(TARGET_DIR)$@")

# Usage: $(call quiet-command,command and args,"NAME","args to print")
# This will run "command and args", and either:
#  if V=1 just print the whole command and args
#  otherwise print the 'quiet' output in the format "  NAME     args to print"
# NAME should be a short name of the command, 7 letters or fewer.
# If called with only a single argument, will print nothing in quiet mode.
quiet-command-run = $(if $(V),,$(if $2,printf "  %-7s %s\n" $2 $3 && ))$1
quiet-@ = $(if $(V),,@)
quiet-command = $(quiet-@)$(call quiet-command-run,$1,$2,$3)

# cc-option
# Usage: CFLAGS+=$(call cc-option, -falign-functions=0, -malign-functions=0)

cc-option = $(if $(shell $(CC) $1 $2 -S -o /dev/null -xc /dev/null \
              >/dev/null 2>&1 && echo OK), $2, $3)
cc-c-option = $(if $(shell $(CC) $1 $2 -c -o /dev/null -xc /dev/null \
                >/dev/null 2>&1 && echo OK), $2, $3)

VPATH_SUFFIXES = %.c %.h %.S %.cc %.cpp %.m %.mak %.texi %.sh %.rc Kconfig% %.json.in
set-vpath = $(if $1,$(foreach PATTERN,$(VPATH_SUFFIXES),$(eval vpath $(PATTERN) $1)))

# install-prog list, dir
define install-prog
	$(INSTALL_DIR) "$2"
	$(INSTALL_PROG) $1 "$2"
	$(if $(STRIP),$(STRIP) $(foreach T,$1,"$2/$(notdir $T)"),)
endef

# Logical functions (for operating on y/n values like CONFIG_FOO vars)
# Inputs to these must be either "y" (true) or "n" or "" (both false)
# Output is always either "y" or "n".
# Usage: $(call land,$(CONFIG_FOO),$(CONFIG_BAR))
# Logical NOT
lnot = $(if $(subst n,,$1),n,y)
# Logical AND
land = $(if $(findstring yy,$1$2),y,n)
# Logical OR
lor = $(if $(findstring y,$1$2),y,n)
# Logical XOR (note that this is the inverse of leqv)
lxor = $(if $(filter $(call lnot,$1),$(call lnot,$2)),n,y)
# Logical equivalence (note that leqv "","n" is true)
leqv = $(if $(filter $(call lnot,$1),$(call lnot,$2)),y,n)
# Logical if: like make's $(if) but with an leqv-like test
lif = $(if $(subst n,,$1),$2,$3)

# String testing functions: inputs to these can be any string;
# the output is always either "y" or "n". Leading and trailing whitespace
# is ignored when comparing strings.
# String equality
eq = $(if $(subst $2,,$1)$(subst $1,,$2),n,y)
# String inequality
ne = $(if $(subst $2,,$1)$(subst $1,,$2),y,n)
# Emptiness/non-emptiness tests:
isempty = $(if $1,n,y)
notempty = $(if $1,y,n)

# Generate files with tracetool
TRACETOOL=$(PYTHON) $(SRC_PATH)/scripts/tracetool.py

.PHONY: clean-timestamp
clean-timestamp:
	rm -f *.timestamp
clean: clean-timestamp

# will delete the target of a rule if commands exit with a nonzero exit status
.DELETE_ON_ERROR:
