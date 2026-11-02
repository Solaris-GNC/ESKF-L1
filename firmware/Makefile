# Firmware del Viradinha MKII: pruebas en el PC, experimento FMA, compilación Xtensa y Arduino.
# Ver COMPILAR.md. Nada de aquí sube nada a ningún sitio.
CC      ?= gcc
PYTHON  ?= python3
V       := viradinha_nav/src
NAVS    := $(V)/nav/eskf.c $(V)/nav/navigation.c $(V)/nav/nav_events.c $(V)/nav/imu_conv.c
VNC     := $(V)/vn/vn_logfmt.c $(V)/vn/vn_params.c $(V)/vn/vn_bench.c
HDRS    := $(wildcard $(V)/nav/*.h $(V)/vn/*.h)
WARN    := -Wall -Wextra -Wpedantic -Wshadow -Werror
CFLAGS  ?= -std=c99 -O2 $(WARN)
FLOAT   := -DESKF_USE_FLOAT -Wdouble-promotion
BUILD   := build
C_DIR   := ../c

# Herramientas de la placa (COMPILAR.md). Valores de este entorno de trabajo; cambiarlos con
# make arduino ARDUINO_CLI=... etc.
FW_DEPS      ?= /home/claude/fw_deps
XTENSA_GCC   ?= $(FW_DEPS)/toolchain/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc
ARDUINO_CLI  ?= $(FW_DEPS)/cli/arduino-cli --config-file $(FW_DEPS)/arduino-cli.yaml
FQBN         ?= arduino:esp32:nano_nora
SPARKFUN_LIB ?= $(FW_DEPS)/libs/SparkFun_ICM-20948_ArduinoLibrary
CTAGS_PROP   ?= --build-property runtime.tools.ctags.path=/usr/bin
ARDUINO_OUT  ?= $(BUILD)/arduino

.PHONY: all test test-c test-tools fma xtensa arduino docs docs-check clean

all: $(BUILD)/test_vn $(BUILD)/test_vn_float $(BUILD)/gen_vnlog $(BUILD)/dump_params

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_vn: test/test_vn.c $(NAVS) $(VNC) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) -I$(V)/nav test/test_vn.c $(VNC) $(NAVS) -lm -o $@
$(BUILD)/test_vn_float: test/test_vn.c $(NAVS) $(VNC) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) $(FLOAT) -I$(V)/nav test/test_vn.c $(VNC) $(NAVS) -lm -o $@
# gen_vnlog emula el firmware: float, como en la placa
$(BUILD)/gen_vnlog: test/gen_vnlog.c $(NAVS) $(VNC) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) $(FLOAT) -I$(V)/nav test/gen_vnlog.c $(VNC) $(NAVS) -lm -o $@
# misma emulación con contracción FMA (a·b+c con un redondeo), sólo en x86-64 con FMA
$(BUILD)/gen_vnlog_fma: test/gen_vnlog.c $(NAVS) $(VNC) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) $(FLOAT) -mfma -ffp-contract=fast -I$(V)/nav test/gen_vnlog.c $(VNC) $(NAVS) -lm -o $@
$(BUILD)/dump_params: test/dump_params.c $(VNC) $(NAVS) $(HDRS) | $(BUILD)
	$(CC) $(CFLAGS) -I$(V)/nav test/dump_params.c $(VNC) $(NAVS) -lm -o $@

HAVE_FMA := $(shell grep -qw fma /proc/cpuinfo 2>/dev/null && echo 1)
GEN_FMA  := $(if $(HAVE_FMA),$(BUILD)/gen_vnlog_fma,)

test: test-c test-tools docs-check

test-c: $(BUILD)/test_vn $(BUILD)/test_vn_float
	$(MAKE) -C $(C_DIR) fw-check
	./$(BUILD)/test_vn
	./$(BUILD)/test_vn_float

test-tools: $(BUILD)/gen_vnlog $(BUILD)/dump_params $(GEN_FMA)
	$(MAKE) -C $(C_DIR) build/nav_replay build/nav_replay_float
	$(PYTHON) test/test_tools_fw.py --gen $(BUILD)/gen_vnlog --dump $(BUILD)/dump_params \
	  --nav-replay $(C_DIR)/build/nav_replay --nav-replay-float $(C_DIR)/build/nav_replay_float \
	  $(if $(GEN_FMA),--gen-fma $(GEN_FMA),)

# Experimento de tolerancias placa/PC (COMPARACION.md): FMA frente a sin FMA, 9 escenarios
fma: $(BUILD)/gen_vnlog_fma
	$(MAKE) -C $(C_DIR) build/nav_replay_float
	$(PYTHON) test/fma_experiment.py --gen-fma $(BUILD)/gen_vnlog_fma --nav-replay-float $(C_DIR)/build/nav_replay_float

# Compilación estricta del C puro con el compilador de la placa (sin Arduino) y pila por función
xtensa: | $(BUILD)
	mkdir -p $(BUILD)/xtensa
	cd $(BUILD)/xtensa && for f in $(addprefix ../../,$(NAVS) $(VNC)); do \
	  $(XTENSA_GCC) -std=c99 -Os -mlongcalls $(WARN) -Wdouble-promotion -DARDUINO_ARCH_ESP32 \
	    -fstack-usage -c $$f || exit 1; done
	@echo "pila por función (Xtensa, -Os, bytes) — las 12 mayores:"
	@cat $(BUILD)/xtensa/*.su | sort -t'	' -k2 -n -r | head -12 | sed 's/^/  /'

# Sketch completo con arduino-cli, en los tres perfiles, con -fstack-usage
arduino:
	for P in 0 1 2; do \
	  $(ARDUINO_CLI) compile -b $(FQBN) --library $(SPARKFUN_LIB) --build-path $(ARDUINO_OUT)/p$$P \
	    --warnings all $(CTAGS_PROP) \
	    --build-property "compiler.c.extra_flags=-fstack-usage -DVN_PROFILE=$$P" \
	    --build-property "compiler.cpp.extra_flags=-fstack-usage -DVN_PROFILE=$$P" \
	    -v viradinha_nav > $(ARDUINO_OUT)/p$$P.log 2>&1 || { tail -30 $(ARDUINO_OUT)/p$$P.log; exit 1; }; \
	  echo "perfil $$P:"; grep -E "Sketch uses|Global variables" $(ARDUINO_OUT)/p$$P.log | sed 's/^/  /'; \
	  echo "  avisos: $$(grep -c 'warning:' $(ARDUINO_OUT)/p$$P.log)"; done

# Tabla de parámetros de PARAMETROS.md generada desde vn_params.c
docs: $(BUILD)/dump_params
	$(PYTHON) test/update_docs.py PARAMETROS.md $(BUILD)/dump_params

docs-check: $(BUILD)/dump_params
	$(PYTHON) test/update_docs.py PARAMETROS.md $(BUILD)/dump_params --check

clean:
	rm -rf $(BUILD)
