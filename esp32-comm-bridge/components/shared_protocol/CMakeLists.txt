# The UART frame protocol as a real ESP-IDF component.
#
# Why a component instead of a file in src/:
#   ESP-IDF compiles whatever sits next to main.cpp in a component's src/
#   directory with C++ flags, and shared/protocol.c is C. Putting the source in
#   its own component lets CMake hand it to the C compiler with the flags it
#   needs, so there is exactly one copy of the protocol in the repository — no
#   generated mirror, and nothing to keep in sync.
#
# Path depth: this component lives at
#   esp32-comm-bridge/components/shared_protocol/
# so three levels up is the repository root, where shared/ lives.
idf_component_register(
    SRCS "../../../shared/protocol.c"
    INCLUDE_DIRS "../../../shared"
)
