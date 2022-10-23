MIDDLEWARE_LOCATION = ../SWSD004

USER_APP_C_SOURCES += \
	$(MIDDLEWARE_LOCATION)/geolocation_middleware/common/mw_common.c \
	$(MIDDLEWARE_LOCATION)/geolocation_middleware/gnss/src/gnss_helpers.c \
	$(MIDDLEWARE_LOCATION)/geolocation_middleware/gnss/src/gnss_queue.c \
	$(MIDDLEWARE_LOCATION)/geolocation_middleware/gnss/src/gnss_middleware.c \
	$(MIDDLEWARE_LOCATION)/geolocation_middleware/wifi/src/wifi_helpers.c \
	$(MIDDLEWARE_LOCATION)/geolocation_middleware/wifi/src/wifi_middleware.c

COMMON_C_INCLUDES += \
	-I$(MIDDLEWARE_LOCATION)/geolocation_middleware/bsp \
	-I$(MIDDLEWARE_LOCATION)/geolocation_middleware/common  \
	-I$(MIDDLEWARE_LOCATION)/geolocation_middleware/gnss/src \
	-I$(MIDDLEWARE_LOCATION)/geolocation_middleware/wifi/src

ifeq ($(APP_TRACE),yes)
COMMON_C_DEFS += \
    -DMW_DBG_TRACE=1
endif