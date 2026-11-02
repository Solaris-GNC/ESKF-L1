# Servicio ESKF para SPP: pruebas en el PC, compilación Xtensa y compilación ESP-IDF.
# SPP es una referencia de SÓLO LECTURA: se compila tal cual desde SPP_ROOT (commit 9247d9e).
# Nada de aquí publica, modifica SPP ni toca el anfitrión.
#
#   make test                      PC: núcleo (float y double), router SPP real, herramientas y escenarios
#   make flight                    resumen por fases de los escenarios de vuelo sintéticos
#   make xtensa                    compilador de la placa: avisos y pila por función
#   make idf                       ESP-IDF real con idf_test_app (componentes eskf_service y spp)
#   make docs                      tabla de parámetros en docs/PARAMETROS_SPP.md
# Variables: SPP_ROOT, XTENSA_GCC, IDF_PATH, IDF_PYTHON, DUMP_TXT (opcional, volcado recibido).

CC      ?= gcc
CXX     ?= g++
PYTHON  ?= python3
C_DIR   := ../../c
CORE    := $(C_DIR)/eskf.c $(C_DIR)/navigation.c $(C_DIR)/nav_events.c
SRC     := src/eskf_service.c src/eskf_svc_params.c src/eskf_svc_msg.c
GLUE    := src/spp_services_eskf.c
HDRS    := $(wildcard include/*.h) $(wildcard $(C_DIR)/*.h) test/test_common.h
WARN    := -Wall -Wextra -Wpedantic -Wshadow -Werror
NOFAST  := -fno-fast-math -ffp-contract=off
CFLAGS  ?= -O2
FLOAT   := -DESKF_USE_FLOAT -Wdouble-promotion
STRICT  := -Wconversion -Wno-sign-conversion
INC     := -Iinclude -I$(C_DIR)
BUILD   := build

SPP_ROOT ?= $(abspath ../../../solaris-packet-protocol)
SPP_INC  := $(BUILD)/spp_inc
SPP_SRC  := core/core.c core/commonbit.c core/pubsub/pubsub.c services/databank/databank.c \
            services/log/log.c util/crc.c hal/hal.c hal/time/time.c hal/spi/spi.c
SPP_OBJS := $(addprefix $(BUILD)/spp/,$(SPP_SRC:.c=.o))

FW_DEPS    ?= $(abspath ../../../../fw_deps)
XTENSA_GCC ?= $(FW_DEPS)/toolchain/xtensa-esp32s3-elf/bin/xtensa-esp32s3-elf-gcc
IDF_PATH   ?= $(abspath ../../../../idf/esp-idf-v4.4.7)
IDF_PYTHON ?= $(abspath ../../../../idf/venv/bin/python)

.PHONY: all test test-c test-spp test-tools flight xtensa idf docs spp-check clean

all: $(BUILD)/test_service $(BUILD)/test_service_double $(BUILD)/svc_replay $(BUILD)/svc_replay_fma $(BUILD)/test_spp_router

$(BUILD):
	mkdir -p $(BUILD)

spp-check:
	@test -f $(SPP_ROOT)/core/pubsub/pubsub.c || { echo "BLOQUEO: no está SPP en SPP_ROOT=$(SPP_ROOT)"; \
	  echo "  descomprimir solaris-packet-protocol-9247d9e8df75ec62cd42fa5c42f5d0f660479a67.zip ahí o pasar SPP_ROOT=..."; exit 1; }

# raíz de inclusión que contiene «spp/» (como en el CMakeLists.txt de SPP), sin copiar nada
$(SPP_INC)/spp: | $(BUILD) spp-check
	mkdir -p $(SPP_INC)
	ln -sfn $(SPP_ROOT) $(SPP_INC)/spp

$(BUILD)/spp/%.o: $(SPP_INC)/spp
	@mkdir -p $(dir $@)
	$(CC) -std=c11 $(CFLAGS) $(NOFAST) -I$(SPP_INC) -c $(SPP_INC)/spp/$*.c -o $@

$(BUILD)/test_service: test/test_service.c $(SRC) $(CORE) $(HDRS) | $(BUILD)
	$(CC) -std=c99 $(CFLAGS) $(WARN) $(NOFAST) $(FLOAT) $(INC) test/test_service.c $(SRC) $(CORE) -lm -pthread -o $@
$(BUILD)/test_service_double: test/test_service.c $(SRC) $(CORE) $(HDRS) | $(BUILD)
	$(CC) -std=c99 $(CFLAGS) $(WARN) $(NOFAST) $(INC) test/test_service.c $(SRC) $(CORE) -lm -pthread -o $@
# el anfitrión de prueba usa float, como la placa
$(BUILD)/svc_replay: test/svc_replay.c $(SRC) $(CORE) $(HDRS) | $(BUILD)
	$(CC) -std=c99 $(CFLAGS) $(WARN) $(NOFAST) $(FLOAT) $(INC) test/svc_replay.c $(SRC) $(CORE) -lm -o $@
# variante con contracción a FMA (otra aritmética): sólo para justificar las tolerancias placa–PC
$(BUILD)/svc_replay_fma: test/svc_replay.c $(SRC) $(CORE) $(HDRS) | $(BUILD)
	$(CC) -std=c99 $(CFLAGS) $(WARN) -ffp-contract=fast -mfma $(FLOAT) $(INC) test/svc_replay.c $(SRC) $(CORE) -lm -o $@
$(BUILD)/test_spp_router: test/test_spp_router.c $(GLUE) $(SRC) $(CORE) $(HDRS) $(SPP_OBJS)
	$(CC) -std=c11 $(CFLAGS) $(WARN) $(NOFAST) $(FLOAT) $(INC) -I$(SPP_INC) test/test_spp_router.c $(GLUE) $(SRC) $(CORE) \
	  $(SPP_OBJS) -lm -o $@

test: test-c test-spp test-tools

test-c: $(BUILD)/test_service $(BUILD)/test_service_double $(SPP_INC)/spp
	@echo "== compilación estricta del código propio (C99 núcleo, C11 pegamento SPP, float y double)"
	for f in $(SRC); do $(CC) -std=c99 $(CFLAGS) $(WARN) $(STRICT) $(FLOAT) $(INC) -c $$f -o $(BUILD)/strict.o || exit 1; \
	  $(CC) -std=c99 $(CFLAGS) $(WARN) $(STRICT) $(INC) -c $$f -o $(BUILD)/strict.o || exit 1; done
	$(CC) -std=c11 $(CFLAGS) $(WARN) $(STRICT) $(FLOAT) $(INC) -I$(SPP_INC) -c $(GLUE) -o $(BUILD)/strict.o
	$(CXX) -std=c++11 $(WARN) -DESKF_USE_FLOAT $(INC) -I$(SPP_INC) -c test/cpp_header_check.cpp -o $(BUILD)/cpp_header_check.o
	./$(BUILD)/test_service
	./$(BUILD)/test_service_double

test-spp: $(BUILD)/test_spp_router
	@echo "== integración con el router REAL de SPP (un proceso por escenario)"
	@fail=0; for s in $$(./$(BUILD)/test_spp_router); do ./$(BUILD)/test_spp_router $$s || fail=1; done; exit $$fail

test-tools: $(BUILD)/svc_replay $(BUILD)/svc_replay_fma $(BUILD)/test_spp_router
	$(MAKE) -C $(C_DIR) build/nav_replay_float
	$(PYTHON) test/test_spp_tools.py --svc $(BUILD)/svc_replay --nav-replay $(C_DIR)/build/nav_replay_float \
	  --router $(BUILD)/test_spp_router --svc-fma $(BUILD)/svc_replay_fma $(if $(DUMP_TXT),--dump-txt $(DUMP_TXT),) \
	  $(if $(RESULTADOS),--resultados $(RESULTADOS),)

# escenarios de vuelo sintéticos (OpenRocket «Simulation 6») con su resumen por fases
flight: $(BUILD)/svc_replay
	$(PYTHON) tools/flight_check.py --svc $(BUILD)/svc_replay test/data/openrocket_sim6_nominal.csv \
	  test/data/openrocket_sim6_adverso.csv

# Código propio y núcleo con el compilador de la placa (estricto) y pila por función
xtensa: $(SPP_INC)/spp
	@test -x $(XTENSA_GCC) || { echo "BLOQUEO: falta XTENSA_GCC=$(XTENSA_GCC)"; exit 1; }
	mkdir -p $(BUILD)/xtensa
	cd $(BUILD)/xtensa && for f in $(addprefix ../../,$(SRC) $(GLUE) $(CORE)); do \
	  $(XTENSA_GCC) -std=gnu11 -Os -mlongcalls $(WARN) $(NOFAST) -Wdouble-promotion -DESKF_USE_FLOAT \
	    -I../../include -I../../$(C_DIR) -I../spp_inc -fstack-usage -c $$f || exit 1; done
	@echo "pila por función (Xtensa, -Os, bytes): servicio, pegamento y las mayores"
	@cat $(BUILD)/xtensa/*.su | grep -E "eskf_svc|sppEskf|handle_|build_|fill_output|track|SPP_SERVICES_ESKF" | \
	  sort -t'	' -k2 -n -r | sed 's/^/  /'
	@cat $(BUILD)/xtensa/*.su | sort -t'	' -k2 -n -r | head -5 | sed 's/^/  /'

# Compilación ESP-IDF real: idf_test_app compila SPP (sin cambios) y este componente
idf: $(SPP_INC)/spp
	@test -d $(IDF_PATH) || { echo "BLOQUEO: falta IDF_PATH=$(IDF_PATH)"; exit 1; }
	rm -rf $(BUILD)/idf && mkdir -p $(BUILD)/idf
	cd $(BUILD)/idf && IDF_PATH=$(IDF_PATH) IDF_COMPONENT_MANAGER=0 SPP_ROOT=$(SPP_ROOT) \
	  SPP_INC_ROOT=$(abspath $(SPP_INC)) PATH=$(dir $(XTENSA_GCC)):$$PATH cmake -G Ninja -DIDF_TARGET=esp32s3 \
	  -DPYTHON=$(IDF_PYTHON) -DPYTHON_DEPS_CHECKED=1 ../../idf_test_app > cmake.log 2>&1 || { tail -20 cmake.log; exit 1; }
	cd $(BUILD)/idf && IDF_PATH=$(IDF_PATH) IDF_COMPONENT_MANAGER=0 SPP_ROOT=$(SPP_ROOT) \
	  SPP_INC_ROOT=$(abspath $(SPP_INC)) PATH=$(dir $(XTENSA_GCC)):$$PATH ninja > ninja.log 2>&1 || { tail -30 ninja.log; exit 1; }
	@echo "avisos en el código propio: $$(grep -E 'warning:' $(BUILD)/idf/ninja.log | grep -c -E 'eskf_service|spp_services_eskf|/c/(eskf|navigation|nav_events)\.c' || true)"
	@echo "avisos totales (incluido SPP sin modificar): $$(grep -c 'warning:' $(BUILD)/idf/ninja.log || true)"
	@tail -1 $(BUILD)/idf/ninja.log

docs: $(BUILD)/svc_replay
	$(PYTHON) tools/update_params_md.py docs/PARAMETROS_SPP.md $(BUILD)/svc_replay

clean:
	rm -rf $(BUILD)
