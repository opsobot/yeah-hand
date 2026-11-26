// Yeah Hand firmware version.
// Change FW_VERSION for every published release: same text as the GitHub release tag
// (one numbering for the whole repository: hardware, firmware, documents).
// FACTORY STATUS also prints the build date and the first 16 hex digits of the
// application's ELF SHA-256 (computed by the ESP-IDF build): it identifies the exact
// build even when FW_VERSION was not changed.
#pragma once
#define FW_VERSION "v2.0"
