
# ---- Compiler and tools --------------------------------------------------- #
CC      := gcc
CFLAGS  := -Wall -Wextra -O2 -I.
AR      := ar
ARFLAGS := rcs

# ---- Source structure ----------------------------------------------------- #
SRC := vec.c mat.c quat.c
OBJ := $(SRC:.c=.o)
LIB := mathlib.a

# ---- Default target ------------------------------------------------------- #
all: $(LIB)

# ---- Build static library ------------------------------------------------- #
$(LIB): $(OBJ)
	@echo "📚 Archiving $@..."
	$(AR) $(ARFLAGS) $@ $^

# ---- Compile objects ------------------------------------------------------ #
%.o: %.c mathlib.h
	@echo "🧱 Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# ---- Cleaning rules ------------------------------------------------------- #
clean:
	@echo "🧹 Cleaning object files..."
	@rm -f $(OBJ)

fclean: clean
	@echo "🗑️  Removing static library..."
	@rm -f $(LIB)

re: fclean all

.PHONY: all clean fclean re

