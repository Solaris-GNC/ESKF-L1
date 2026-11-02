/* setup() y loop() del sketch independiente (ver viradinha_nav.ino). */
#include <Arduino.h>

#include "src/vn/vn_app.h"

void setup() { vn_app_setup(); }

void loop() { vn_app_loop(); }
