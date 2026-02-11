# SSD Simulator Makefile

CC = gcc
CFLAGS = -Wall -Wextra -O2 -g -pthread
INCLUDES = -I./include

# Directories
SRC_DIR = src
OBJ_DIR = obj
BIN_DIR = bin
TEST_DIR = tests

# Source files
COMMON_SRCS = $(SRC_DIR)/common/task_scheduler.c \
              $(SRC_DIR)/common/inter_core_comm.c \
              $(SRC_DIR)/common/ddr_manager.c \
              $(SRC_DIR)/common/log_manager.c

BACKEND_SRCS = $(SRC_DIR)/backend/nand_simulator.c \
               $(SRC_DIR)/backend/pcie_controller.c

MODULES_SRCS = $(SRC_DIR)/modules/l2p_manager.c \
               $(SRC_DIR)/modules/nvme_controller.c \
               $(SRC_DIR)/modules/cache_manager.c \
               $(SRC_DIR)/modules/gc_manager.c

DRIVER_SRCS = $(SRC_DIR)/drivers/block_device.c

MAIN_SRC = $(SRC_DIR)/ssd_simulator.c

TEST_SRCS = $(SRC_DIR)/tests/test_task_scheduler.c

# Object files
COMMON_OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(COMMON_SRCS))
BACKEND_OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(BACKEND_SRCS))
MODULES_OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(MODULES_SRCS))
DRIVER_OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(DRIVER_SRCS))
TEST_OBJS = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(TEST_SRCS))
ALL_OBJS = $(COMMON_OBJS) $(BACKEND_OBJS) $(MODULES_OBJS) $(DRIVER_OBJS)

# Targets
all: dirs $(BIN_DIR)/ssd_simulator $(BIN_DIR)/test_task_scheduler

dirs:
	@mkdir -p $(OBJ_DIR)/common $(OBJ_DIR)/backend $(OBJ_DIR)/modules $(OBJ_DIR)/drivers $(OBJ_DIR)/tests $(BIN_DIR)

# Pattern rules
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Main SSD Simulator
$(BIN_DIR)/ssd_simulator: $(MAIN_SRC) $(ALL_OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) $^ -o $@

# Test: Task Scheduler
$(BIN_DIR)/test_task_scheduler: $(OBJ_DIR)/tests/test_task_scheduler.o $(OBJ_DIR)/common/task_scheduler.o
	$(CC) $(CFLAGS) $^ -o $@

# Individual component tests
test-scheduler: $(BIN_DIR)/test_task_scheduler
	@echo "Running Task Scheduler Tests..."
	@./$(BIN_DIR)/test_task_scheduler

test-ssd: $(BIN_DIR)/ssd_simulator
	@echo "Running SSD Simulator Self Test..."
	@./$(BIN_DIR)/ssd_simulator test

benchmark: $(BIN_DIR)/ssd_simulator
	@echo "Running SSD Simulator Benchmark..."
	@./$(BIN_DIR)/ssd_simulator benchmark

test-all: test-scheduler test-ssd

# Clean
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Debug build
debug: CFLAGS = -Wall -Wextra -O0 -g -pthread -DDEBUG
debug: all

# Release build
release: CFLAGS = -Wall -Wextra -O3 -pthread -DNDEBUG
release: all

.PHONY: all dirs clean debug release test-scheduler test-integration test-all
