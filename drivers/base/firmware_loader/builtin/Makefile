# This file implements the GNOME Build API:
# http://people.gnome.org/~walters/docs/build-api.txt

FIRMWAREDIR = /lib/firmware

all:

FWNAME    = $(patsubst $(obj)/%.gen.S,%,$@)
FWSTR     = $(subst $(comma),_,$(subst /,_,$(subst .,_,$(subst -,_,$(FWNAME)))))
ASM_WORD  = $(if $(CONFIG_64BIT),.quad,.long)
ASM_ALIGN = $(if $(CONFIG_64BIT),3,2)
PROGBITS  = $(if $(CONFIG_ARM),%,@)progbits

install:
	mkdir -p $(DESTDIR)$(FIRMWAREDIR)
	cp -rv * $(DESTDIR)$(FIRMWAREDIR)
	rm -rf $(DESTDIR)$(FIRMWAREDIR)/usbdux
	find $(DESTDIR)$(FIRMWAREDIR) \( -name 'WHENCE' -or -name 'LICENSE.*' -or \
		-name 'LICENCE.*' \) -exec rm -- {} \;
