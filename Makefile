# defines
CC = gcc
OBJS = main.o igc_user_space.o atemsys_main.o igc_main.o igc_mac.o igc_i225.o igc_base.o igc_nvm.o igc_phy.o
CFLAGS = -c -O2 -Wall
SCR_DIR = src
OBJ_DIR = obj
VPATH = $(SCR_DIR)/igc:$(SCR_DIR)  # tell compiler where to look for source files to compile
OBJ_OUT = $(patsubst %.o, $(OBJ_DIR)/%.o, $(OBJS))
TARGET = igc_user_space

.PHONY: all clean

all: pre-build $(TARGET)

pre-build:
	mkdir -p $(OBJ_DIR)

# link and produce executable
$(TARGET): $(OBJ_OUT)
	$(CC) $^ -o $@

# compile object files
$(OBJ_DIR)/%.o: %.c
	$(CC) $(CFLAGS) $< -o $@

# delete all output
clean:
	rm -rf $(OBJ_DIR)
	rm -rf $(TARGET)


# Notes:
#
# $^ : all source files
# $< : first source file
# $@ : target name

# -c : compile but don't link
# -o : output name