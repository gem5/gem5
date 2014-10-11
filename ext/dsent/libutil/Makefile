
# Define the directories that will be compiled
DIRS_TO_COMPILE := . \

DIRS = $(patsubst %,$(CURDIR)/%,$(DIRS_TO_COMPILE))

SRCS = $(foreach dir, $(DIRS), $(wildcard $(dir)/*.cc))

OBJS = $(SRCS:%.cc=%.o)

DEF_FLAGS = 

ifdef LIBUTIL_IS_LOG
	LIBUTIL_IS_LOG = true
else
	LIBUTIL_IS_LOG = false
endif
DEF_FLAGS += -DLIBUTIL_IS_LOG=$(LIBUTIL_IS_LOG)

INCLUDE_FLAGS = $(foreach dir, $(DIRS), -I$(dir))
OPT_FLAGS = -O2 -g 
WARN_FLAGS = -pedantic -Wall -W -Wextra -Werror
CXXFLAGS = $(OPT_FLAGS) $(WARN_FLAGS) $(INCLUDE_FLAGS) $(DEF_FLAGS)

TARGET = $(CURDIR)/libutil.a

all: $(TARGET)

$(TARGET): $(OBJS)
	ar rcs $@ $^
#$(TARGET): $(OBJS)
#	$(CXX) $(CXXFLAGS) $^ -o $(TARGET)

%.o: %.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

%/created:
	mkdir -p $(dir $@)
	touch $@

clean:
	$(RM) -rf $(OBJS) $(TARGET)

