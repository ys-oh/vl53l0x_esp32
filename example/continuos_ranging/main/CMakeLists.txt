
file (GLOB sources *.c *.cpp)

idf_component_register(SRCS ${sources}
                    INCLUDE_DIRS ".")

