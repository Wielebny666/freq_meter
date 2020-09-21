set(SOURCES freq_meter.c)

idf_component_register(SRCS "${SOURCES}"
					INCLUDE_DIRS .)
