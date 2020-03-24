# Elevon: Triple-Modular Redundant Fly-by-Wire FCC and ARINC 664 Avionics Network
# Star Language: Cangjie with C17 hardware acceleration fastpath

CC ?= clang
CFLAGS ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -pedantic -std=c17
CFLAGS += -Iinclude -Isrc/c

AR ?= ar
ARFLAGS ?= rcs

BIN_DIR = bin
BUILD_DIR = build
SRC_C = $(wildcard src/c/*.c)
SRC_CLI = $(wildcard src/cli/*.c)

OBJ_C = $(patsubst src/c/%.c, $(BUILD_DIR)/%.o, $(SRC_C))
OBJ_CLI = $(patsubst src/cli/%.c, $(BUILD_DIR)/%.o, $(SRC_CLI))

TARGET_LIB = $(BUILD_DIR)/libelevon.a
TARGET_CLI = $(BIN_DIR)/elevon

TEST_SRCS = $(wildcard tests/test_*.c)
TEST_BINS = $(patsubst tests/%.c, $(BIN_DIR)/%, $(TEST_SRCS))

.PHONY: all clean test check-em-dash dirs bench

all: dirs $(TARGET_LIB) $(TARGET_CLI)

dirs:
	@mkdir -p $(BIN_DIR) $(BUILD_DIR)

$(BUILD_DIR)/%.o: src/c/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: src/cli/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET_LIB): $(OBJ_C) | dirs
	@if [ -n "$(OBJ_C)" ]; then \
		$(AR) $(ARFLAGS) $@ $^; \
	else \
		touch $@; \
	fi

$(TARGET_CLI): $(OBJ_CLI) $(TARGET_LIB) | dirs
	@if [ -n "$(OBJ_CLI)" ]; then \
		$(CC) $(CFLAGS) $(OBJ_CLI) $(TARGET_LIB) -o $@ -lpthread -lm; \
	fi

test: dirs $(TARGET_LIB) $(TEST_BINS)
	@echo "=== Verifying No Em-Dash Typography Invariant ==="
	@python3 tools/lint_no_em_dash.py
	@if [ -n "$(TEST_BINS)" ]; then \
		echo "=== Running C Unit Tests ==="; \
		for t in $(TEST_BINS); do \
			echo "Running $$t..."; \
			$$t || exit 1; \
		done; \
	fi
	@if [ -d "tests" ] && [ -n "$$(ls -A tests/test_*.py 2>/dev/null)" ]; then \
		echo "=== Running Python Integration Tests ==="; \
		python3 -m unittest discover -s tests -p "test_*.py" -v || exit 1; \
	fi

$(BIN_DIR)/test_%: tests/test_%.c $(TARGET_LIB) | dirs
	$(CC) $(CFLAGS) $< $(TARGET_LIB) -o $@ -lpthread -lm

check-em-dash:
	@python3 tools/lint_no_em_dash.py

clean:
	rm -rf $(BIN_DIR) $(BUILD_DIR) *.fdr *.journal *.sock *.pcap
