# Compiler and flags
CC      = gcc
CFLAGS  = -Wall -Wextra -O2 -I.
AR      = ar
ARFLAGS = rcs

# Source and object files
SRC = vec.c mat.c quat.c ekf_utils.c
OBJ = $(SRC:.c=.o)

# Output static library
LIB = libmathlib.a

# Default target
all: $(LIB)

# Build the static library from object files
$(LIB): $(OBJ)
	$(AR) $(ARFLAGS) $@ $^

# Compile .c to .o
%.o: %.c mathlib.h
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJ) $(LIB)

.PHONY: all clean

