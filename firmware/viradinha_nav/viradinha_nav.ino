/*
 * viradinha_nav.ino — Viradinha MKII: adquisición, estimación (ESKF), telemetría y registro.
 * Sin control ni aerofrenos.
 *
 * Perfil: src/vn/vn_profile.h (por defecto ADQUISICIÓN). Parámetros: src/vn/vn_params.c.
 * Núcleo del estimador: src/nav/ (copia de c/ — `make -C c fw-sync`; `make -C c fw-check`
 * comprueba que coinciden). Instrucciones: firmware/COMPILAR.md.
 *
 * setup() y loop() están en vn_main.cpp: este .ino no define funciones, así el preprocesador
 * de Arduino no genera prototipos (compila igual con su ctags y con ctags genéricos).
 * Para integrarlo en el firmware del equipo: llamar a vn_app_setup() / vn_app_loop() desde
 * vuestro setup()/loop(), borrar vn_main.cpp y sustituir los ganchos débiles de vn_app.h
 * (sumidero del registro y telemetría).
 */
#include "src/vn/vn_app.h"
