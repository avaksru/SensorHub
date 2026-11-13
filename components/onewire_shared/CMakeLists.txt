idf_component_register(
    SRCS "onewire_shared.cpp"
    INCLUDE_DIRS "include"
    REQUIRES app_config onewire_bus
)