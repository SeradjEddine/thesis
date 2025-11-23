# ---- Compiler and flags ---------------------------------------------------- #
CC      := cc
CFLAGS  := -Wall -Werror -Wextra -pthread -I./include -I./src/mathlib
LDLIBS  := -lm 

# ---- Project structure ----------------------------------------------------- #
NAME        := fusion_sim
SRC_DIR     := src
BUILD_DIR   := build
MATHLIB_DIR := $(SRC_DIR)/mathlib
MATHLIB     := $(MATHLIB_DIR)/mathlib.a
TARGET := $(BUILD_DIR)/$(NAME)

# ---- Source files ---------------------------------------------------------- #
SRCS := consumer.c \
		consumer_utils.c \
        ekf.c \
		ekf_utils.c \
        fuzzy_supervisor.c \
		fuzzy_supervisor_utils.c \
        logger.c \
        main.c \
        producer.c \
        ringbuffer.c \
        sensor_reader.c \
        multi_fuse_utils.c \
	multi_fuse.c
# Map sources to object files in build/
OBJS := $(addprefix $(BUILD_DIR)/, $(SRCS:.c=.o))

# ---- Rules ----------------------------------------------------------------- #

all: $(TARGET)

# Build final binary
$(TARGET): $(MATHLIB) $(OBJS)
	@echo "🔗 Linking $(NAME)..."
	$(CC) $(CFLAGS) $(OBJS) $(MATHLIB) $(LDLIBS) -o $@
	@echo "✅ Build complete!"

# Build mathlib first
$(MATHLIB):
	@echo "📦 Building mathlib..."
	@$(MAKE) -C $(MATHLIB_DIR)

# Compile object files into build/ directory
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	@echo "🧱 Compiling $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# ---- Clean ----------------------------------------------------------------- #
clean:
	@echo "🧹 Cleaning object files..."
	@$(MAKE) -C $(MATHLIB_DIR) clean
	@rm -rf $(BUILD_DIR)/*.o

fclean: clean
	@echo "🗑️ Removing $(NAME) and mathlib..."
	@$(MAKE) -C $(MATHLIB_DIR) fclean
	@rm -f $(NAME)

re: fclean all

# ---- Phony targets --------------------------------------------------------- #
.PHONY: all clean fclean re
