CC      ?= gcc
CXX     ?= g++
PYTHON  ?= python3
WARN    := -Wall -Wextra -Wpedantic -Wshadow -Werror
CFLAGS  ?= -std=c99 -O2 $(WARN)
STRICT  := -Wconversion -Wno-sign-conversion
FLOAT   := -DESKF_USE_FLOAT -Wdouble-promotion
BUILD   := build
REF     ?=
REFARG  := $(if $(REF),--ref-dir $(REF),)

SRCS  := eskf.c navigation.c nav_events.c imu_conv.c
HDRS  := eskf.h navigation.h nav_events.h imu_conv.h test/test_util.h
BINS  := test_eskf test_nav test_fw test_sim replay nav_replay

.PHONY: all test test-c test-tools compare check bench stack fw-sync fw-check clean

all: $(foreach b,$(BINS),$(BUILD)/$(b) $(BUILD)/$(b)_float)

$(BUILD):
	mkdir -p $(BUILD)

define BIN
$(BUILD)/$(1): $(SRCS) $(HDRS) test/$(1).c | $(BUILD)
	$$(CC) $$(CFLAGS) -I. -Itest $(SRCS) test/$(1).c -lm -o $$@
$(BUILD)/$(1)_float: $(SRCS) $(HDRS) test/$(1).c | $(BUILD)
	$$(CC) $$(CFLAGS) $$(FLOAT) -I. -Itest $(SRCS) test/$(1).c -lm -o $$@
endef
$(foreach b,$(BINS) bench,$(eval $(call BIN,$(b))))

# Compilación estricta (C99 double/float y C++) y pruebas en C
test-c: $(foreach b,test_eskf test_nav test_fw test_sim,$(BUILD)/$(b) $(BUILD)/$(b)_float)
	for f in $(SRCS); do \
	  $(CC) $(CFLAGS) $(STRICT) -I. -c $$f -o $(BUILD)/strict.o && \
	  $(CC) $(CFLAGS) $(STRICT) $(FLOAT) -I. -c $$f -o $(BUILD)/strict_f.o && \
	  $(CXX) -std=c++11 $(WARN) -x c++ -I. -c $$f -o $(BUILD)/strict_cpp.o || exit 1; done
	./$(BUILD)/test_eskf
	./$(BUILD)/test_eskf_float
	./$(BUILD)/test_nav
	./$(BUILD)/test_nav_float
	./$(BUILD)/test_fw
	./$(BUILD)/test_fw_float
	./$(BUILD)/test_sim
	./$(BUILD)/test_sim_float

test-tools: $(BUILD)/replay $(BUILD)/nav_replay
	$(PYTHON) test/test_tools.py $(BUILD)/replay --nav-replay $(BUILD)/nav_replay $(REFARG)

test: test-c test-tools fw-check

compare: $(BUILD)/replay $(BUILD)/replay_float
	$(PYTHON) test/compare_with_python.py $(BUILD)/replay --seeds 8 $(REFARG)
	$(PYTHON) test/compare_with_python.py $(BUILD)/replay_float --seeds 3 --tol 1e-2 --psd-tol 1e-5 $(REFARG)

check: test compare

bench: $(BUILD)/bench $(BUILD)/bench_float
	./$(BUILD)/bench
	./$(BUILD)/bench_float

stack: | $(BUILD)
	cd $(BUILD) && $(CC) -std=c99 -O2 -fstack-usage -I.. -c $(addprefix ../,$(SRCS)) && cat *.su > double.su
	cd $(BUILD) && $(CC) -std=c99 -O2 $(FLOAT) -fstack-usage -I.. -c $(addprefix ../,$(SRCS)) && cat eskf.su navigation.su nav_events.su imu_conv.su > float.su
	@echo "Pila por función en ESTE ordenador; para la placa: make -C ../firmware stack-xtensa"
	@sort -t'	' -k2 -n -r $(BUILD)/float.su | head -6 | sed 's/^/  float   /'

# El firmware lleva una COPIA de estos archivos (Arduino sólo compila la carpeta del sketch).
# fw-sync la actualiza; fw-check falla si difieren (se ejecuta en `make test`).
FW_SRC := ../firmware/viradinha_nav/src/nav
fw-sync:
	mkdir -p $(FW_SRC) && cp $(SRCS) eskf.h navigation.h nav_events.h imu_conv.h $(FW_SRC)/
fw-check:
	@if [ -d $(FW_SRC) ]; then for f in $(SRCS) eskf.h navigation.h nav_events.h imu_conv.h; do \
	  cmp -s $$f $(FW_SRC)/$$f || { echo "fw-check: $(FW_SRC)/$$f difiere de c/$$f (make fw-sync)"; exit 1; }; done; \
	  echo "fw-check: la copia del firmware coincide con c/"; else echo "fw-check: sin firmware junto a c/ (omitido)"; fi

clean:
	rm -rf $(BUILD)
