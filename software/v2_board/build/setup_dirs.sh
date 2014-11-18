cat Makefile  | grep '^src/\|^FreeRTOS/\|^MyDrivers/' | grep -v '%' | grep -v ':' | awk '{print $1}' | while read F; do dirname "$F"; done | sort | uniq | while read F; do echo "$F" ; mkdir -p "$F" ; done
touch makedep.mk
